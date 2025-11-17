#include "ch.h"
#include "hal.h"
#include "shell.h"
#include <chprintf.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cfloat>

#include "sampleStream/USBStream.h"
#include "util/asservMath.h"
#include "util/chibiOsAllocatorWrapper.h"
#include "AsservMain.h"
#include "commandManager/CommandManager.h"
#include "SpeedController/SpeedController.h"
#include "SpeedController/AdaptativeSpeedController.h"
#include "robotStub/MotorEncoderSimulator.h"
#include "Odometry.h"
#include "AccelerationLimiter/SimpleAccelerationLimiter.h"
#include "AccelerationLimiter/AdvancedAccelerationLimiter.h"
#include "AccelerationLimiter/AccelerationDecelerationLimiter.h"
#include "Pll.h"
#include "blockingDetector/OldSchoolBlockingDetector.h"
#include "Communication/SerialCbor.h"
#include "sampleStream/configuration/ConfigurationHandler.h"
#include "sampleStream/commands/CommandHandler.h"


#define ASSERV_THREAD_FREQUENCY (200)
#define ASSERV_THREAD_PERIOD_S (1.0/ASSERV_THREAD_FREQUENCY)
#define ASSERV_POSITION_DIVISOR (5)

#define ENCODERS_WHEELS_RADIUS_MM (31.40/2.0)
#define ENCODERS_WHEELS_DISTANCE_MM (261.2)
#define ENCODERS_TICKS_BY_TURN (1440*4)

#define MAX_SPEED_MM_PER_SEC (1500)

#define DIST_REGULATOR_KP (5)
#define DIST_REGULATOR_MAX_ACC_FW (1200)
#define DIST_REGULATOR_MAX_DEC_FW (1200)
#define DIST_REGULATOR_MAX_ACC_BW (1200)
#define DIST_REGULATOR_MAX_DEC_BW (1200)
#define ACC_DEC_DAMPLING (1.6)


#define ANGLE_REGULATOR_KP (800)
#define ANGLE_REGULATOR_MAX_ACC (3000)


float speed_controller_right_Kp[NB_PI_SUBSET] = { 0.1, 0.1, 0.1};
float speed_controller_right_Ki[NB_PI_SUBSET] = { 1.0, 0.8, 0.6};
float speed_controller_right_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60};

float speed_controller_left_Kp[NB_PI_SUBSET] = { 0.1, 0.1, 0.1};
float speed_controller_left_Ki[NB_PI_SUBSET] = { 1.0, 0.8, 0.6};
float speed_controller_left_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60};

#define PLL_BANDWIDTH (150)

#define BLOCKING_DETECTOR_ANGLE_SPEED_THRESHOLD (M_PI/32)
#define BLOCKING_DETECTOR_DIST_SPEED_THRESHOLD (5)
#define BLOCKING_DETECTOR_BLOCKING_DURATION_THRESHOLD (0.5)


#define COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD (0.02)
#define COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm (2.5)
#define COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD (M_PI/8)
#define COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm (25)
#define COMMAND_MANAGER_ALIGN_ONLY_EXIT_ANGLE_THRESHOLD_RAD COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD/10


#define COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm (3)
Goto::GotoConfiguration preciseGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm};

#define COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm (20)
Goto::GotoConfiguration waypointGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm};

#define COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD (M_PI/2)
GotoNoStop::GotoNoStopConfiguration gotoNoStopConf = {COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD, (150/DIST_REGULATOR_KP), 85 };


MotorEncoderSimulator *motorEncoder;
Regulator *angleRegulator;
Regulator *distanceRegulator;

Odometry *odometry;

AdaptativeSpeedController *speedControllerRight;
AdaptativeSpeedController *speedControllerLeft;

Pll *rightPll;
Pll *leftPll;

OldSchoolBlockingDetector *blockingDetector = nullptr;

SimpleAccelerationLimiter *angleAccelerationlimiter;
//AdvancedAccelerationLimiter *distanceAccelerationLimiter;
AccelerationDecelerationLimiter *distanceAccelerationLimiter;

CommandManager *commandManager;
AsservMain *mainAsserv;

SerialCbor *esp32IoCbor;

ConfigurationHandler *configurationHandler;

CommandHandler *commandHandler;



static void initAsserv()
{
    odometry = new Odometry (ENCODERS_WHEELS_DISTANCE_MM, 0, 0);

    motorEncoder = new MotorEncoderSimulator(ASSERV_THREAD_PERIOD_S, ENCODERS_WHEELS_RADIUS_MM, ENCODERS_TICKS_BY_TURN, odometry);

    angleRegulator = new Regulator(ANGLE_REGULATOR_KP, MAX_SPEED_MM_PER_SEC);
    distanceRegulator = new Regulator(DIST_REGULATOR_KP, FLT_MAX);

    rightPll = new Pll (PLL_BANDWIDTH);
    leftPll = new Pll(PLL_BANDWIDTH);


    speedControllerRight = new AdaptativeSpeedController(speed_controller_right_Kp, speed_controller_right_Ki, speed_controller_right_SpeedRange, 100, FLT_MAX, ASSERV_THREAD_FREQUENCY);
    speedControllerLeft = new AdaptativeSpeedController(speed_controller_left_Kp, speed_controller_left_Ki, speed_controller_left_SpeedRange, 100, FLT_MAX, ASSERV_THREAD_FREQUENCY);


    angleAccelerationlimiter = new SimpleAccelerationLimiter(ANGLE_REGULATOR_MAX_ACC);


    distanceAccelerationLimiter = new AccelerationDecelerationLimiter(DIST_REGULATOR_MAX_ACC_FW, DIST_REGULATOR_MAX_DEC_FW, DIST_REGULATOR_MAX_ACC_BW, DIST_REGULATOR_MAX_DEC_BW, MAX_SPEED_MM_PER_SEC, ACC_DEC_DAMPLING, DIST_REGULATOR_KP);

    commandManager = new CommandManager( COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm, COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD,
                                   preciseGotoConf, waypointGotoConf, gotoNoStopConf,
                                   *angleRegulator, *distanceRegulator,
                                   MAX_SPEED_MM_PER_SEC, MAX_SPEED_MM_PER_SEC/2,
                                   distanceAccelerationLimiter);

    mainAsserv = new AsservMain( ASSERV_THREAD_FREQUENCY, ASSERV_POSITION_DIVISOR,
                           ENCODERS_WHEELS_RADIUS_MM, ENCODERS_WHEELS_DISTANCE_MM, ENCODERS_TICKS_BY_TURN,
                           *commandManager, *motorEncoder, *motorEncoder, *odometry,
                           *angleRegulator, *distanceRegulator,
                           *angleAccelerationlimiter, *distanceAccelerationLimiter,
                           *speedControllerRight, *speedControllerLeft,
                           *rightPll, *leftPll,
                           blockingDetector);
                           
    esp32IoCbor = new SerialCbor(&LPSD1, *odometry, *commandManager, *motorEncoder, *mainAsserv);

    configurationHandler = new ConfigurationHandler (angleRegulator, distanceRegulator, angleAccelerationlimiter, distanceAccelerationLimiter, speedControllerRight, speedControllerLeft);

    commandHandler = new CommandHandler(*commandManager, *mainAsserv);
}

void serialIoWrapperPositionOutput(void *)
{
    esp32IoCbor->positionOutput();
}


void serialIoWrapperCommandInput(void *)
{
    esp32IoCbor->commandInput();
}


/*
 *  As the dynamic allocation is disabled after init,
 *  use this semaphore to ensure that init is finished before
 *  disabling further dynamic allocation
 */
static binary_semaphore_t asservStarted_semaphore;

static THD_WORKING_AREA(waAsservThread, 512);
static THD_FUNCTION(AsservThread, arg)
{
    (void) arg;
    chRegSetThreadName("AsservThread");

    USBStream::init(nullptr, ASSERV_THREAD_FREQUENCY, configurationHandler, commandHandler);

    chBSemSignal(&asservStarted_semaphore);

    mainAsserv->mainLoop();
}

static THD_WORKING_AREA(waLowPrioUSBThread, 512);
static THD_FUNCTION(LowPrioUSBThread, arg)
{
    (void) arg;
    chRegSetThreadName("LowPrioUSBThread");


    while (!chThdShouldTerminateX())
    {
       USBStream::instance()->USBStreamHandleConnection_lowerpriothread();
    }

}


THD_WORKING_AREA(wa_shell, 1024);

char history_buffer[SHELL_MAX_HIST_BUFF];
char *completion_buffer[SHELL_MAX_COMPLETIONS];

float config_buffer[30];
void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv);

THD_WORKING_AREA(wa_raspioInput, 1024);
THD_WORKING_AREA(wa_raspioOutput, 1024);

BaseSequentialStream *outputStream;
int main(void)
{
    halInit();
    chSysInit();

    initAsserv();

    /*  
    * There's an internal pulldown activated at reset on pin PB4 which is an encoder input !
    *  Write the bit UCPD1_DBDIS in PWR_CR3 to disable this pulldown
    */

    PWR->CR3 |= (1<<14);


    /* PA6 (not used) is connected to PA15 (PWM output) in the board, so force PA6 to float to avoid any problem.
    * Same thing for PA5 and PB7 but both are unconnected
    */
    palSetPadMode(GPIOA, 6, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING );
    palSetPadMode(GPIOB, 7, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING );


    /*
    * USART 1:  For communication with the brain µc
    * RX: PA10
    * TX: PA9
    */
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
    sdStart(&SD1, NULL);


    /*
    *  LPUSART 1 : built-in usb serial port. For shell only
    */
    sdStart(&LPSD1, NULL);

    outputStream = reinterpret_cast<BaseSequentialStream*>(&SD1);
    shellInit();

    chBSemObjectInit(&asservStarted_semaphore, true);
    chThdCreateStatic(waAsservThread, sizeof(waAsservThread), HIGHPRIO, AsservThread, NULL);
    chBSemWait(&asservStarted_semaphore);

    chThdCreateStatic(waLowPrioUSBThread, sizeof(waLowPrioUSBThread), LOWPRIO, LowPrioUSBThread, NULL);


    // Custom commands
    const ShellCommand shellCommands[] = { { "asserv", &(asservCommandUSB) }, { nullptr, nullptr } };
    ShellConfig shellCfg =
    {
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

    thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO+1, shellThread, &shellCfg);
    chRegSetThreadNameX(shellThd, "shell");

    /* 
     *  Needed thread to run SerialIO (ie: here, communication with ESP32).
     *  C wrapping function are needed to bridge through C and C++
     */
    thread_t *threadPositionOutput = chThdCreateStatic(wa_raspioInput, sizeof(wa_raspioInput), LOWPRIO+3, serialIoWrapperPositionOutput, nullptr);
    chRegSetThreadNameX(threadPositionOutput, "positionOutput");
    thread_t *threadCommandInput = chThdCreateStatic(wa_raspioOutput, sizeof(wa_raspioOutput), LOWPRIO+4, serialIoWrapperCommandInput, nullptr);
    chRegSetThreadNameX(threadCommandInput, "commandInput");

    deactivateHeapAllocation();

    chThdSetPriority(LOWPRIO);
    while (true)
    {
        chThdSleepMilliseconds(1000);
    }
}

void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv)
{
    auto printUsage = []()
    {
        chprintf(outputStream,"Usage :");
        chprintf(outputStream," - asserv wheelcalib \r\n");
        chprintf(outputStream," - asserv enablemotor 0|1\r\n");
        chprintf(outputStream," - asserv enablepolar 0|1\r\n");
        chprintf(outputStream," - asserv coders \r\n");
        chprintf(outputStream," - asserv reset \r\n");
        chprintf(outputStream," - asserv motorspeed [r|l] speed \r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv wheelspeedstep [r|l] [speed] [step time] \r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv robotfwspeedstep [speed] [step time] \r\n");
        chprintf(outputStream," - asserv robotangspeedstep [speed] [step time] \r\n");
        chprintf(outputStream," - asserv speedcontrol [r|l] [Kp] [Ki] \r\n");
        chprintf(outputStream," - asserv angleacc delta_speed \r\n");
        chprintf(outputStream," - asserv distacc delta_speed \r\n");
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
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv pll freq\r\n");
    };
    (void) chp;

    if (argc == 0)
    {
        printUsage();
        return;
    }

     if (!strcmp(argv[0], "speedcontrol"))
    {
        char side = *argv[1];
        float Kp = atof(argv[2]);
        float Ki = atof(argv[3]);
        uint8_t range = atof(argv[4]);;

        chprintf(outputStream, "setting speed control Kp:%.2f Ki:%.2f range:%d to side %c \r\n", Kp, Ki, range, side);

        if (side == 'r')
            speedControllerRight->setGains(Kp, Ki, range);
        else if (side == 'l')
            speedControllerLeft->setGains(Kp, Ki, range);
    }
    else if (!strcmp(argv[0], "angleacc"))
    {
        float acc = atof(argv[1]);
        chprintf(outputStream, "setting angle acceleration limit to %.2f \r\n", acc);

        angleAccelerationlimiter->setMaxAcceleration(acc);
    }
    else if (!strcmp(argv[0], "distacc"))
    {
        float acc_max = atof(argv[1]);
        float acc_min = atof(argv[2]);
        float acc_threshold = atof(argv[3]);
        chprintf(outputStream, "setting distance acceleration limiter max %.2f min %.2f threshold %.2f \r\n", acc_max, acc_min, acc_threshold);

//       distanceAccelerationLimiter->setMaxAcceleration(acc_max);
//       distanceAccelerationLimiter->setMinAcceleration(acc_min);
//       distanceAccelerationLimiter->setHighSpeedThreshold(acc_threshold);
    }
    else if (!strcmp(argv[0], "distaccdec"))
    {
        float acc_fw = atof(argv[1]);
        float dec_fw = atof(argv[2]);
        float acc_bw = atof(argv[3]);
        float dec_bw = atof(argv[4]);
        chprintf(outputStream, "setting distance acceleration dec limiter fw : acc%.2f dec%.2f bw: acc%.2f dec%.2f \r\n", acc_fw, dec_fw, acc_bw, dec_bw);

         distanceAccelerationLimiter->setMaxAccFW(acc_fw);
         distanceAccelerationLimiter->setMaxDecFW(dec_fw);
         distanceAccelerationLimiter->setMaxAccBW(acc_bw);
         distanceAccelerationLimiter->setMaxDecBW(dec_bw);
    }
    else if (!strcmp(argv[0], "addangle"))
    {
        float angle = atof(argv[1]);
        chprintf(outputStream, "Adding angle %.2frad \r\n", angle);

        commandManager->addTurn(angle);
    }
    else if (!strcmp(argv[0], "anglereset"))
    {
        chprintf(outputStream, "Reseting angle accumulator \r\n");
        angleRegulator->reset();
    }
    else if (!strcmp(argv[0], "distreset"))
    {
        chprintf(outputStream, "Reseting distance accumulator \r\n");
        distanceRegulator->reset();
    }
    else if (!strcmp(argv[0], "adddist"))
    {
        float dist = atof(argv[1]);

        bool ok = commandManager->addStraightLine(dist);
        chprintf(outputStream, "Adding distance %.2fmm %d\r\n", dist, ok );

    }
    else if (!strcmp(argv[0], "anglecontrol"))
    {
        float Kp = atof(argv[1]);
        chprintf(outputStream, "setting angle Kp to %.2f \r\n", Kp);

        angleRegulator->setGain(Kp);
    }
    else if (!strcmp(argv[0], "distcontrol"))
    {
        float Kp = atof(argv[1]);
        chprintf(outputStream, "setting dist Kp to %.2f \r\n", Kp);

        distanceRegulator->setGain(Kp);
    }
    else if (!strcmp(argv[0], "enablemotor"))
    {
        bool enable = !(atoi(argv[1]) == 0);
        chprintf(outputStream, "%s motor output\r\n", (enable ? "enabling" : "disabling"));
        mainAsserv->enableMotors(enable);
    }
    else if (!strcmp(argv[0], "coders"))
    {
//        chprintf(outputStream, "Encoders count %d %d \r\n", encoders->getRightEncoderTotalCount(), encoders->getLeftEncoderTotalCount());
    }
    else if (!strcmp(argv[0], "reset"))
    {
        mainAsserv->reset();
        chprintf(outputStream, "asserv resetted \r\n");
    }
    else if (!strcmp(argv[0], "motorspeed"))
    {
        char side = *argv[1];
        float speedGoal = atof(argv[2]);

        chprintf(outputStream, "setting wheel %s to speed %.2f \r\n", (side == 'r') ? "right" : "left", speedGoal);

//        if (side == 'l')
//            md22MotorController->setMotorLeftSpeed(speedGoal);
//        else
//            md22MotorController->setMotorRightSpeed(speedGoal);
    }
    else if (!strcmp(argv[0], "addgoto"))
    {
        float X = atof(argv[1]);
        float Y = atof(argv[2]);
        chprintf(outputStream, "Adding goto(%.2f,%.2f) consign\r\n", X, Y);

        commandManager->addGoTo(X, Y);
    }
    else if (!strcmp(argv[0], "pll"))
    {
        float bw = atof(argv[1]);
        chprintf(outputStream, "Set PLL bandwidth to %.2f \r\n", bw);
        rightPll->setBandwidth(bw);
        leftPll->setBandwidth(bw);
        rightPll->reset();
        leftPll->reset();
    }
    else if (!strcmp(argv[0], "damp"))
    {
        float value = atof(argv[1]);
        chprintf(outputStream, "Set acce/dec limiter dampling factor to %.2f \r\n", value);
        // distanceAccelerationLimiter->setDamplingFactor(value);
    }
    else if (!strcmp(argv[0], "gototest"))
    {
        
        chprintf(outputStream, "commandManager->addGoTo(1000, 0);\r\n");
        commandManager->addGoTo(1000, 0);
        // chThdSleepMilliseconds(1200);
        // commandManager->setEmergencyStop();


        // commandManager->addGoToNoStop(500, 0);
        // commandManager->addGoToNoStop(500, 300);
        // commandManager->addGoToNoStop(0, 300);
        // commandManager->addGoToNoStop(0, 0);
        // commandManager->addGoToAngle(1000, 0);

//        commandManager->addGoToWaypointBack(-800, 0);
//        commandManager->addGoToWaypointBack(-800, -300);
//        commandManager->addGoToWaypointBack( 0, -300);
//        commandManager->addGoToWaypointBack( 0, 0);
//        commandManager->addGoToAngle(1000, 0);

//        commandManager->addGoToWaypoint(800, 0);
//        commandManager->addGoToWaypoint(800, 300);
//        commandManager->addGoToWaypoint(0, 300);
//        commandManager->addGoToWaypoint(0, 0);
//        commandManager->addGoToAngle(1000, 0);
//
//        commandManager->addGoTo(500, 0);
//        commandManager->addGoTo(500, 300);
//        commandManager->addGoTo(0, 300);
//        commandManager->addGoTo(0, 0);
//        commandManager->addGoToAngle(1000, 0);



#if 0
    commandManager->addGoTo(800,200);
    commandManager->addGoTo(720,280);
    commandManager->addGoTo(720,370);
    commandManager->addGoTo(710,380);
    commandManager->addGoTo(710,600);
    commandManager->addGoTo(610,700);
    commandManager->addGoTo(540,700);
    commandManager->addGoTo(540,700);
    commandManager->addGoTo(490,650);
    commandManager->addGoTo(350,650);
    commandManager->addGoTo(250,550);
    commandManager->addGoTo(250,450);
    commandManager->addGoTo(700,450);
#endif

#if 0
    commandManager->addGoToWaypoint(800,200);
    commandManager->addGoToWaypoint(720,280);
    commandManager->addGoToWaypoint(720,370);
    commandManager->addGoToWaypoint(710,380);
    commandManager->addGoToWaypoint(710,600);
    commandManager->addGoToWaypoint(610,700);
    commandManager->addGoToWaypoint(540,700);
    commandManager->addGoToWaypoint(540,700);
    commandManager->addGoToWaypoint(490,650);
    commandManager->addGoToWaypoint(350,650);
    commandManager->addGoToWaypoint(250,550);
    commandManager->addGoToWaypoint(250,450);
    commandManager->addGoToWaypoint(700,450);
#endif

#if 0
    commandManager->addGoToNoStop(800,200);
    commandManager->addGoToNoStop(720,280);
    commandManager->addGoToNoStop(720,370);
    commandManager->addGoToNoStop(710,380);
    commandManager->addGoToNoStop(710,600);
    commandManager->addGoToNoStop(610,700);
    commandManager->addGoToNoStop(540,700);
    commandManager->addGoToNoStop(540,700);
    commandManager->addGoToNoStop(490,650);
    commandManager->addGoToNoStop(350,650);
    commandManager->addGoToNoStop(250,550);
    commandManager->addGoToNoStop(250,450);
    commandManager->addGoToNoStop(250,1000);
#endif



    }
    else
    {
        printUsage();
    }
}


