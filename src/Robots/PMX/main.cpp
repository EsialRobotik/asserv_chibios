#include "ch.h"
#include "hal.h"
#include "shell.h"
#include <chprintf.h>
#include <cstdlib>
#include <cstring>

#include "util/asservMath.h"
#include "util/chibiOsAllocatorWrapper.h"
#include "AsservMain.h"
#include "commandManager/CommandManager.h"
#include "Encoders/QuadratureEncoder.h"
#include "motorController/Md22.h"
#include "Odometry.h"
#include "USBStream.h"
#include "SlopeFilter.h"
#include "Pll.h"

#define ASSERV_THREAD_FREQUENCY (500)
#define ASSERV_THREAD_PERIOD_S (1.0/ASSERV_THREAD_FREQUENCY)
#define ASSERV_POSITION_DIVISOR (5)

#define ENCODERS_WHEELS_RADIUS (47.2/2.0)
#define ENCODERS_WHEELS_DISTANCE_MM (297)
#define ENCODERS_TICKS_BY_TURN (1024*4)

#define MAX_SPEED (500)

#define DIST_REGULATOR_KP (9)
#define DIST_REGULATOR_MAX_DELTA (8/ASSERV_THREAD_PERIOD_S)
#define ANGLE_REGULATOR_KP (1400)
#define ANGLE_REGULATOR_MAX_DELTA (8/ASSERV_THREAD_PERIOD_S)

float speed_controller_right_Kp[NB_PI_SUBSET] = { 0.25, 0.25, 0.25 };
float speed_controller_right_Ki[NB_PI_SUBSET] = { 0.45, 0.45, 0.45 };
float speed_controller_right_speed_set[NB_PI_SUBSET] = { 500.0, 500.0, 500.0 };

float speed_controller_left_Kp[NB_PI_SUBSET] = { 0.25, 0.25, 0.25 };
float speed_controller_left_Ki[NB_PI_SUBSET] = { 0.45, 0.45, 0.45 };
float speed_controller_left_speed_set[NB_PI_SUBSET] = { 500.0, 500.0, 500.0 };

#define PLL_BANDWIDTH (250)

#define COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD (0.1)
#define COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm (1)
#define COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD (M_PI/8)
#define COMMAND_MANAGER_GOTO_CHAIN_NEXT_CMD_DIST_mm (50)

QuadratureEncoder encoders(false, false, 1, 1);
Md22 Md22MotorController(true, false);

Regulator angleRegulator(ANGLE_REGULATOR_KP, MAX_SPEED);
Regulator distanceRegulator(DIST_REGULATOR_KP, MAX_SPEED);

Odometry odometry(ENCODERS_WHEELS_DISTANCE_MM, 0, 0);

SpeedController speedControllerRight(speed_controller_right_Kp, speed_controller_right_Ki,
        speed_controller_right_speed_set, 100, MAX_SPEED, ASSERV_THREAD_FREQUENCY);
SpeedController speedControllerLeft(speed_controller_left_Kp, speed_controller_left_Ki, speed_controller_left_speed_set,
        100, MAX_SPEED, ASSERV_THREAD_FREQUENCY);

Pll rightPll(PLL_BANDWIDTH);
Pll leftPll(PLL_BANDWIDTH);

SlopeFilter angleSlopeFilter(ANGLE_REGULATOR_MAX_DELTA);
SlopeFilter distSlopeFilter(DIST_REGULATOR_MAX_DELTA);

CommandManager commandManager(COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD,
COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm,
COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_CHAIN_NEXT_CMD_DIST_mm, angleRegulator,
        distanceRegulator);

AsservMain mainAsserv(ASSERV_THREAD_FREQUENCY, ASSERV_POSITION_DIVISOR,
ENCODERS_WHEELS_RADIUS, ENCODERS_WHEELS_DISTANCE_MM, ENCODERS_TICKS_BY_TURN, commandManager, Md22MotorController,
        encoders, odometry, angleRegulator, distanceRegulator, angleSlopeFilter, distSlopeFilter, speedControllerRight,
        speedControllerLeft, rightPll, leftPll);

static THD_WORKING_AREA(waAsservThread, 512);
static THD_FUNCTION(AsservThread, arg)
{
    (void) arg;
    chRegSetThreadName("AsservThread");

    Md22MotorController.init();
    encoders.init();
    encoders.start();
    USBStream::init();

//    while(1) //desactivation du mainloop
//    { chThdSleep(chTimeMS2I(200));}

    mainAsserv.mainLoop();
}

THD_WORKING_AREA(wa_shell, 512);
THD_WORKING_AREA(wa_controlPanel, 512);
THD_FUNCTION(ControlPanelThread, p);
binary_semaphore_t controlPanelSemaphore;

char history_buffer[SHELL_MAX_HIST_BUFF];
char *completion_buffer[SHELL_MAX_COMPLETIONS];

void asservCommand(BaseSequentialStream *chp, int argc, char **argv);

BaseSequentialStream *outputStream;
int main(void)
{

    halInit();
    chSysInit();

    sdStart(&SD2, NULL);
    shellInit();

    outputStream = reinterpret_cast<BaseSequentialStream*>(&SD2);

    // Custom commands
    const ShellCommand shellCommands[] = { { "asserv", &(asservCommand) }, { nullptr, nullptr } };
    ShellConfig shellCfg = {
    /* sc_channel */outputStream,
    /* sc_commands */shellCommands,
#if (SHELL_USE_HISTORY == TRUE)
                              /* sc_histbuf */history_buffer,
                              /* sc_histsize */sizeof(history_buffer),
#endif
#if (SHELL_USE_COMPLETION == TRUE)
                              /* sc_completion */completion_buffer
#endif
            };

    thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, shellThread, &shellCfg);
    chRegSetThreadNameX(shellThd, "shell");

    chThdCreateStatic(waAsservThread, sizeof(waAsservThread), HIGHPRIO, AsservThread, NULL);

    chBSemObjectInit(&controlPanelSemaphore, true);

    chThdSleep(chTimeMS2I(200)); // Histoire d'être sur que l'usb soit lancé une fois que ce thread tourne...
    thread_t *controlPanelThd = chThdCreateStatic(wa_controlPanel, sizeof(wa_controlPanel), LOWPRIO, ControlPanelThread,
            nullptr);
    chRegSetThreadNameX(controlPanelThd, "controlPanel");

    deactivateHeapAllocation();

    chThdSetPriority(LOWPRIO);
    while (true) {
        palClearPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(250);
        palSetPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(250);
    }
}

void asservCommand(BaseSequentialStream *chp, int argc, char **argv)
{
    auto printUsage = []() {
        chprintf(outputStream,"Usage :");
        chprintf(outputStream," - asserv enablemotor 0|1\r\n");
        chprintf(outputStream," - asserv enablepolar 0|1\r\n");
        chprintf(outputStream," - asserv startPanel \r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv wheelspeedstep [r|l] [speed] [step time] \r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv robotfwspeedstep [speed] [step time] \r\n");
        chprintf(outputStream," - asserv robotangspeedstep [speed] [step time] \r\n");
        chprintf(outputStream," - asserv speedcontrol [r|l] [Kp] [Ki] \r\n");
        chprintf(outputStream," - asserv angleSlope delta_speed \r\n");
        chprintf(outputStream," - asserv distSlope delta_speed \r\n");
        chprintf(outputStream," ------------------- \r\n");
        chprintf(outputStream," - asserv addangle angle_rad \r\n");
        chprintf(outputStream," - asserv anglereset\r\n");
        chprintf(outputStream," - asserv anglecontrol Kp\r\n");
        chprintf(outputStream," ------------------- \r\n");
        chprintf(outputStream," - asserv adddist mm \r\n");
        chprintf(outputStream," - asserv distreset\r\n");
        chprintf(outputStream," - asserv distcontrol Kp\r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv addgoto X Y\r\n");
        chprintf(outputStream," - asserv gototest\r\n");
    };
    (void) chp;

    if (argc == 0) {
        printUsage();
        return;
    }

    if (!strcmp(argv[0], "wheelspeedstep")) {
        char side = *argv[1];
        float speedGoal = atof(argv[2]);
        int time = atoi(argv[3]);
        chprintf(outputStream, "setting fw robot speed %.2f rad/s for %d ms\r\n", speedGoal, time);

        chprintf(outputStream, "setting wheel %s to speed %.2f rad/s for %d ms \r\n", (side == 'r') ? "right" : "left",
                speedGoal, time);

        float speedRight = speedGoal;
        float speedLeft = 0;
        if (side == 'l') {
            speedLeft = speedGoal;
            speedRight = 0;
        }

        mainAsserv.setWheelsSpeed(speedRight, speedLeft);
        chThdSleepMilliseconds(time);
        mainAsserv.setWheelsSpeed(0, 0);
    } else if (!strcmp(argv[0], "robotfwspeedstep")) {
        float speedGoal = atof(argv[1]);
        int time = atoi(argv[2]);
        chprintf(outputStream, "setting fw robot speed %.2f rad/s for %d ms\r\n", speedGoal, time);

        mainAsserv.setRegulatorsSpeed(speedGoal, 0);
        chThdSleepMilliseconds(time);
        mainAsserv.setRegulatorsSpeed(0, 0);
    } else if (!strcmp(argv[0], "robotangspeedstep")) {
        float speedGoal = atof(argv[1]);
        int time = atoi(argv[2]);
        chprintf(outputStream, "setting angle robot speed %.2f rad/s for %d ms\r\n", speedGoal, time);

        mainAsserv.setRegulatorsSpeed(0, speedGoal);
        chThdSleepMilliseconds(time);
        mainAsserv.setRegulatorsSpeed(0, 0);
    } else if (!strcmp(argv[0], "speedcontrol")) {
        char side = *argv[1];
        float Kp = atof(argv[2]);
        float Ki = atof(argv[3]);

        chprintf(outputStream, "setting speed control Kp:%.2f Ki:%.2f to side %c \r\n", Kp, Ki, side);

        if (side == 'r')
            speedControllerRight.setGains(Kp, Ki);
        else if (side == 'l')
            speedControllerLeft.setGains(Kp, Ki);
    } else if (!strcmp(argv[0], "angleSlope")) {
        float slope = atof(argv[1]);
        chprintf(outputStream, "setting angle slope delta %.2f \r\n", slope);

        angleSlopeFilter.setSlope(slope);
    } else if (!strcmp(argv[0], "distSlope")) {
        float slope = atof(argv[1]);
        chprintf(outputStream, "setting distance slope delta %.2f \r\n", slope);

        distSlopeFilter.setSlope(slope);
    } else if (!strcmp(argv[0], "addangle")) {
        float angle = atof(argv[1]);
        chprintf(outputStream, "Adding angle %.2frad \r\n", angle);

        mainAsserv.resetToNormalMode();
        commandManager.addTurn(angle);
    } else if (!strcmp(argv[0], "anglereset")) {
        chprintf(outputStream, "Reseting angle accumulator \r\n");
        angleRegulator.reset();
    } else if (!strcmp(argv[0], "distreset")) {
        chprintf(outputStream, "Reseting distance accumulator \r\n");
        distanceRegulator.reset();
    } else if (!strcmp(argv[0], "adddist")) {
        float dist = atof(argv[1]);
        chprintf(outputStream, "Adding distance %.2fmm \r\n", dist);

        mainAsserv.resetToNormalMode();
        commandManager.addStraightLine(dist);
    } else if (!strcmp(argv[0], "anglecontrol")) {
        float Kp = atof(argv[1]);
        chprintf(outputStream, "setting angle Kp to %.2f \r\n", Kp);

        angleRegulator.setGain(Kp);
    } else if (!strcmp(argv[0], "distcontrol")) {
        float Kp = atof(argv[1]);
        chprintf(outputStream, "setting dist Kp to %.2f \r\n", Kp);

        distanceRegulator.setGain(Kp);
    } else if (!strcmp(argv[0], "enablemotor")) {
        bool enable = !(atoi(argv[1]) == 0);
        chprintf(outputStream, "%s motor output\r\n", (enable ? "enabling" : "disabling"));

        mainAsserv.enableMotors(enable);
    } else if (!strcmp(argv[0], "startPanel")) {
        chBSemSignal(&controlPanelSemaphore);
    } else if (!strcmp(argv[0], "enablepolar")) {
        bool enable = !(atoi(argv[1]) == 0);
        chprintf(outputStream, "%s polar control\r\n", (enable ? "enabling" : "disabling"));

        mainAsserv.enablePolar(enable);
    } else if (!strcmp(argv[0], "addgoto")) {
        float X = atof(argv[1]);
        float Y = atof(argv[2]);
        chprintf(outputStream, "Adding goto(%.2f,%.2f) consign\r\n", X, Y);

        mainAsserv.resetToNormalMode();
        commandManager.addGoTo(X, Y);
    } else if (!strcmp(argv[0], "gototest")) {
//                  commandManager.addGoToEnchainement(450,-200);
//                  commandManager.addGoToEnchainement(450,-600);
//                  commandManager.addGoToEnchainement(300,-400);
//                  commandManager.addGoTo(150,0);

        mainAsserv.resetToNormalMode();
        commandManager.addGoToEnchainement(365, -270);
        commandManager.addGoToEnchainement(550, -385);
        commandManager.addGoToEnchainement(490, -590);
        commandManager.addGoToEnchainement(295, -720);
        commandManager.addGoToEnchainement(180, -1000);
        commandManager.addGoToEnchainement(390, -1100);
        commandManager.addGoToEnchainement(550, -900);
        commandManager.addGoToEnchainement(395, -630);
        commandManager.addGoToEnchainement(300, -440);
        commandManager.addGoTo(300, -250);
        commandManager.addGoToAngle(1000, -250);
        commandManager.addStraightLine(-200);

    } else if (!strcmp(argv[0], "md22up")) {
        Md22MotorController.setMotor1Speed(100);
        Md22MotorController.setMotor2Speed(100);
        chprintf(outputStream, "Motors at 100\r\n");

    } else if (!strcmp(argv[0], "md22stop")) {
        Md22MotorController.setMotor1Speed(0);
        Md22MotorController.setMotor2Speed(0);
        chprintf(outputStream, "Motors at 0\r\n");

    } else if (!strcmp(argv[0], "encodervalues")) {
        int16_t encoderDeltaRight_tmp = 0;
        int16_t encoderDeltaLeft_tmp = 0;
        encoders.getValuesAndReset(&encoderDeltaRight_tmp, &encoderDeltaLeft_tmp);

        chprintf(outputStream, "encoders L %d  R %d \r\n", encoderDeltaRight_tmp, encoderDeltaLeft_tmp);
    } else {
        printUsage();
    }
}

THD_FUNCTION(ControlPanelThread, p)
{
    (void) p;
    void *ptr = nullptr;
    uint32_t size = 0;
    char *firstArg = nullptr;
    char *argv[7];
    chBSemWait(&controlPanelSemaphore);
    while (!chThdShouldTerminateX()) {
        USBStream::instance()->getFullBuffer(&ptr, &size);
        if (size > 0) {
            char *buffer = (char*) ptr;

            /*
             *  On transforme la commande recu dans une version argv/argc
             *    de manière a utiliser les commandes shell déjà définie...
             */
            bool prevWasSpace = false;
            firstArg = buffer;
            int nb_arg = 0;
            for (uint32_t i = 0; i < size; i++) {
                if (prevWasSpace && buffer[i] != ' ') {
                    argv[nb_arg++] = &buffer[i];
                }

                if (buffer[i] == ' ' || buffer[i] == '\r' || buffer[i] == '\n') {
                    prevWasSpace = true;
                    buffer[i] = '\0';
                } else {
                    prevWasSpace = false;
                }
            }

            // On évite de faire appel au shell si le nombre d'arg est mauvais ou si la 1ière commande est mauvaise...
            if (nb_arg > 0 && !strcmp(firstArg, "asserv")) {
                asservCommand(nullptr, nb_arg, argv);
            }
            USBStream::instance()->releaseBuffer();
        }
    }
}

