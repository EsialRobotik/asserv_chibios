#include "ch.h"
#include "hal.h"
#include "shell.h"
#include <chprintf.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cfloat>

#include "sampleStream/USBStream.h"
#include "raspIO.h"
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
#include "sampleStream/configuration/ConfigurationHandler.h"
#include "sampleStream/commands/CommandHandler.h"




#define ASSERV_THREAD_FREQUENCY (200)
#define ASSERV_THREAD_PERIOD_S (1.0/ASSERV_THREAD_FREQUENCY)
#define ASSERV_POSITION_DIVISOR (5)

#define ENCODERS_WHEELS_RADIUS_MM (31.40/2.0)
#define ENCODERS_WHEELS_DISTANCE_MM (261.2)
#define ENCODERS_TICKS_BY_TURN (1440*4)

#define REGULATOR_MAX_SPEED_MM_PER_SEC (1500)

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

ConfigurationHandler *configurationHandler;

CommandHandler *commandHandler;

static void initAsserv()
{
    odometry = new Odometry (ENCODERS_WHEELS_DISTANCE_MM, 0, 0);

    motorEncoder = new MotorEncoderSimulator(ASSERV_THREAD_PERIOD_S, ENCODERS_WHEELS_RADIUS_MM, ENCODERS_TICKS_BY_TURN, odometry );

    angleRegulator = new Regulator(ANGLE_REGULATOR_KP, REGULATOR_MAX_SPEED_MM_PER_SEC);
    distanceRegulator = new Regulator(DIST_REGULATOR_KP, FLT_MAX);

    rightPll = new Pll (PLL_BANDWIDTH);
    leftPll = new Pll(PLL_BANDWIDTH);

    speedControllerRight = new AdaptativeSpeedController(speed_controller_right_Kp, speed_controller_right_Ki, speed_controller_right_SpeedRange, 100, FLT_MAX, ASSERV_THREAD_FREQUENCY);
    speedControllerLeft = new AdaptativeSpeedController(speed_controller_left_Kp, speed_controller_left_Ki, speed_controller_left_SpeedRange, 100, FLT_MAX, ASSERV_THREAD_FREQUENCY);


    angleAccelerationlimiter = new SimpleAccelerationLimiter(ANGLE_REGULATOR_MAX_ACC);


    distanceAccelerationLimiter = new AccelerationDecelerationLimiter(DIST_REGULATOR_MAX_ACC_FW, DIST_REGULATOR_MAX_DEC_FW, DIST_REGULATOR_MAX_ACC_BW, DIST_REGULATOR_MAX_DEC_BW, REGULATOR_MAX_SPEED_MM_PER_SEC, ACC_DEC_DAMPLING, DIST_REGULATOR_KP);

//    blockingDetector = new OldSchoolBlockingDetector(ASSERV_THREAD_PERIOD_S, *md22MotorController, *odometry,
//           BLOCKING_DETECTOR_ANGLE_SPEED_THRESHOLD, BLOCKING_DETECTOR_DIST_SPEED_THRESHOLD, BLOCKING_DETECTOR_BLOCKING_DURATION_THRESHOLD);

    commandManager = new CommandManager( COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm, COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD,
                                   preciseGotoConf, waypointGotoConf, gotoNoStopConf,
                                   *angleRegulator, *distanceRegulator,
                                    REGULATOR_MAX_SPEED_MM_PER_SEC, REGULATOR_MAX_SPEED_MM_PER_SEC/2,
                                   distanceAccelerationLimiter);

    mainAsserv = new AsservMain( ASSERV_THREAD_FREQUENCY, ASSERV_POSITION_DIVISOR,
                           ENCODERS_WHEELS_RADIUS_MM, ENCODERS_WHEELS_DISTANCE_MM, ENCODERS_TICKS_BY_TURN,
                           *commandManager, *motorEncoder, *motorEncoder, *odometry,
                           *angleRegulator, *distanceRegulator,
                           *angleAccelerationlimiter, *distanceAccelerationLimiter,
                           *speedControllerRight, *speedControllerLeft,
                           *rightPll, *leftPll,
                           blockingDetector);


    configurationHandler = new ConfigurationHandler (angleRegulator, distanceRegulator, angleAccelerationlimiter, distanceAccelerationLimiter, speedControllerRight, speedControllerLeft);

    commandHandler = new CommandHandler(*commandManager, *mainAsserv);

}



/*
 *  As the dynamic allocation is disabled after init,
 *  use this semaphore to ensure that init is finished before
 *  disabling further dynamic allocation
 */
static binary_semaphore_t asservStarted_semaphore;

static THD_WORKING_AREA(waAsservThread, 1024*4);
static THD_FUNCTION(AsservThread, arg)
{
    (void) arg;
    chRegSetThreadName("AsservThread");

    USBStream::UsbStreamPinConf_t usbPinConf =
    {
        .dataPlusPin_GPIObase = GPIOA, .dataPlusPin_number = 12, .dataPlusPin_alternate = 10,
        .dataMinusPin_GPIObase = GPIOA, .dataMinusPin_number = 11, .dataMinusPin_alternate = 10
    };
    USBStream::init(&usbPinConf, ASSERV_THREAD_FREQUENCY, configurationHandler, commandHandler);

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


THD_WORKING_AREA(wa_shell, 2048);
THD_WORKING_AREA(wa_controlPanel, 256);
//THD_FUNCTION(ControlPanelThread, p);

char history_buffer[SHELL_MAX_HIST_BUFF];
char *completion_buffer[SHELL_MAX_COMPLETIONS];

float config_buffer[30];
void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv);

void asservCommandSerial();


BaseSequentialStream *outputStream;
int main(void)
{
    halInit();
    chSysInit();

    initAsserv();


    sdStart(&SD2, NULL);
    shellInit();

    chBSemObjectInit(&asservStarted_semaphore, true);
    chThdCreateStatic(waAsservThread, sizeof(waAsservThread), HIGHPRIO, AsservThread, NULL);
    chBSemWait(&asservStarted_semaphore);

    chThdCreateStatic(waLowPrioUSBThread, sizeof(waLowPrioUSBThread), LOWPRIO, LowPrioUSBThread, NULL);

    outputStream = reinterpret_cast<BaseSequentialStream*>(&SD2);
    chprintf(outputStream, "Started \r\n");


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

    bool startShell = true;
    if (startShell)
    {
        thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, shellThread, &shellCfg);
        chRegSetThreadNameX(shellThd, "shell");

        // Le thread controlPanel n'a de sens que quand le shell tourne
//        thread_t *controlPanelThd = chThdCreateStatic(wa_controlPanel, sizeof(wa_controlPanel), LOWPRIO, ControlPanelThread, nullptr);
//        chRegSetThreadNameX(controlPanelThd, "controlPanel");
    }

    deactivateHeapAllocation();

    chThdSetPriority(LOWPRIO);
    while (true)
    {
        palClearPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(250);
        palSetPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(250);
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

    if (!strcmp(argv[0], "coders"))
    {
//        chprintf(outputStream, "Encoders count %d %d \r\n", encoders->getRightEncoderTotalCount(), encoders->getLeftEncoderTotalCount());
    }
    else if (!strcmp(argv[0], "reset"))
    {
        mainAsserv->reset();
        chprintf(outputStream, "asserv resetted \r\n");
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
        commandManager->addGoTo(1000, 0);
        chThdSleepMilliseconds(1200);
        commandManager->setEmergencyStop();


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

