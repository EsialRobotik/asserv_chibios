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
#include "Encoders/QuadratureEncoder.h"
#include "motorController/Md22.h"
#include "Odometry.h"
#include "AccelerationLimiter/SimpleAccelerationLimiter.h"
#include "AccelerationLimiter/AccelerationDecelerationLimiter.h"
#include "Pll.h"
#include "blockingDetector/OldSchoolBlockingDetector.h"
#include "config.h"
#include "Communication/RaspIO.h"


float speed_controller_right_Kp[NB_PI_SUBSET] = { SPEED_CTRL_RIGHT_KP_1, SPEED_CTRL_RIGHT_KP_2, SPEED_CTRL_RIGHT_KP_3};
float speed_controller_right_Ki[NB_PI_SUBSET] = { SPEED_CTRL_RIGHT_KI_1, SPEED_CTRL_RIGHT_KI_2, SPEED_CTRL_RIGHT_KI_3};
float speed_controller_right_SpeedRange[NB_PI_SUBSET] = { SPEED_CTRL_RIGHT_SPEED_THRES_1, SPEED_CTRL_RIGHT_SPEED_THRES_2, SPEED_CTRL_RIGHT_SPEED_THRES_3};

float speed_controller_left_Kp[NB_PI_SUBSET] = { SPEED_CTRL_LEFT_KP_1, SPEED_CTRL_LEFT_KP_2, SPEED_CTRL_LEFT_KP_3};
float speed_controller_left_Ki[NB_PI_SUBSET] = { SPEED_CTRL_LEFT_KI_1, SPEED_CTRL_LEFT_KI_2, SPEED_CTRL_LEFT_KI_3};
float speed_controller_left_SpeedRange[NB_PI_SUBSET] = { SPEED_CTRL_LEFT_SPEED_THRES_1, SPEED_CTRL_LEFT_SPEED_THRES_2, SPEED_CTRL_LEFT_SPEED_THRES_3};


Goto::GotoConfiguration preciseGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm};
Goto::GotoConfiguration waypointGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm};
GotoNoStop::GotoNoStopConfiguration gotoNoStopConf = {COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD, (150/DIST_REGULATOR_KP), 85};

RaspIO::AccDecConfiguration normalAccDec =
{
    ANGLE_REGULATOR_MAX_ACC,
    DIST_REGULATOR_MAX_ACC_FW,
    DIST_REGULATOR_MAX_DEC_FW,
    DIST_REGULATOR_MAX_ACC_BW,
    DIST_REGULATOR_MAX_DEC_BW
};


RaspIO::AccDecConfiguration slowAccDec =
{
    ANGLE_REGULATOR_MAX_ACC_SLOW,
    DIST_REGULATOR_MAX_ACC_FW_SLOW,
    DIST_REGULATOR_MAX_DEC_FW_SLOW,
    DIST_REGULATOR_MAX_ACC_BW_SLOW,
    DIST_REGULATOR_MAX_DEC_BW_SLOW
};



Md22::I2cPinInit ESIALCardPinConf_md22 = {GPIOB, 6, GPIOB, 7};
QuadratureEncoder::GpioPinInit ESIALCardPinConf_Encoders = {GPIOC, 6, GPIOA, 7, GPIOA, 1, GPIOA, 0};


QuadratureEncoder *encoders;
Md22 *md22MotorController;

Regulator *angleRegulator;
Regulator *distanceRegulator;

Odometry *odometry;

AdaptativeSpeedController *speedControllerRight;
AdaptativeSpeedController *speedControllerLeft;

Pll *rightPll;
Pll *leftPll;

OldSchoolBlockingDetector *blockingDetector;

SimpleAccelerationLimiter *angleAccelerationlimiter;
AccelerationDecelerationLimiter *distanceAccelerationLimiter;


CommandManager *commandManager;
AsservMain *mainAsserv;

RaspIO *raspIO;

static void initAsserv()
{
    encoders = new QuadratureEncoder(&ESIALCardPinConf_Encoders, false, false, false);
    md22MotorController = new Md22(&ESIALCardPinConf_md22, true, true, false, 100000);

    angleRegulator = new Regulator(ANGLE_REGULATOR_KP, REGULATOR_MAX_SPEED_MM_PER_SEC);
    distanceRegulator = new Regulator(DIST_REGULATOR_KP, FLT_MAX);

    rightPll = new Pll (PLL_BANDWIDTH);
    leftPll = new Pll(PLL_BANDWIDTH);

    odometry = new Odometry (ENCODERS_WHEELS_DISTANCE_MM, 0, 0);

    speedControllerRight = new AdaptativeSpeedController(speed_controller_right_Kp, speed_controller_right_Ki, speed_controller_right_SpeedRange, 100, WHEELS_MAX_SPEED_MM_PER_SEC, ASSERV_THREAD_FREQUENCY);
    speedControllerLeft = new AdaptativeSpeedController(speed_controller_left_Kp, speed_controller_left_Ki, speed_controller_left_SpeedRange, 100, WHEELS_MAX_SPEED_MM_PER_SEC, ASSERV_THREAD_FREQUENCY);


    angleAccelerationlimiter = new SimpleAccelerationLimiter(ANGLE_REGULATOR_MAX_ACC);
    distanceAccelerationLimiter = new AccelerationDecelerationLimiter(DIST_REGULATOR_MAX_ACC_FW, DIST_REGULATOR_MAX_DEC_FW, DIST_REGULATOR_MAX_ACC_BW, DIST_REGULATOR_MAX_DEC_BW, REGULATOR_MAX_SPEED_MM_PER_SEC, ACC_DEC_DAMPLING, DIST_REGULATOR_KP);

    // blockingDetector = new OldSchoolBlockingDetector(ASSERV_THREAD_PERIOD_S, *md22MotorController, *odometry,
    //        BLOCKING_DETECTOR_ANGLE_SPEED_THRESHOLD, BLOCKING_DETECTOR_DIST_SPEED_THRESHOLD, BLOCKING_DETECTOR_BLOCKING_DURATION_THRESHOLD);
    blockingDetector = nullptr;


    commandManager = new CommandManager( COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm, COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD,
                                   preciseGotoConf, waypointGotoConf, gotoNoStopConf,
                                   *angleRegulator, *distanceRegulator,
                                   REGULATOR_MAX_SPEED_MM_PER_SEC, REGULATOR_MAX_SPEED_MM_PER_SEC/3 /* This value should maybe be fine tuned ?*/, 
                                   distanceAccelerationLimiter,
                                   blockingDetector);

    mainAsserv = new AsservMain( ASSERV_THREAD_FREQUENCY, ASSERV_POSITION_DIVISOR,
                           ENCODERS_WHEELS_RADIUS_MM, ENCODERS_WHEELS_DISTANCE_MM, ENCODERS_TICKS_BY_TURN,
                           *commandManager, *md22MotorController, *encoders, *odometry,
                           *angleRegulator, *distanceRegulator,
                           *angleAccelerationlimiter, *distanceAccelerationLimiter,
                           *speedControllerRight, *speedControllerLeft,
                           *rightPll, *leftPll,
                           blockingDetector);


    
    raspIO = new RaspIO(&SD2, *odometry, *commandManager, *md22MotorController, *mainAsserv,
    *angleAccelerationlimiter, *distanceAccelerationLimiter,
    normalAccDec, slowAccDec);
}


void serialIoWrapperPositionOutput(void *)
{
    SerialIO *serialIO = (SerialIO*)raspIO;
    serialIO->positionOutput();
}


void serialIoWrapperCommandInput(void *)
{
    raspIO->commandInput();
}


/*
 *  As the dynamic allocation is disabled after init,
 *  use this semaphore to ensure that init is finished before
 *  disabling further dynamic allocation
 */
static binary_semaphore_t asservStarted_semaphore;

static THD_WORKING_AREA(waAsservThread, 1024);
static THD_FUNCTION(AsservThread, arg)
{
    (void) arg;
    chRegSetThreadName("AsservThread");

    md22MotorController->init();
    encoders->init();
    encoders->start();
    USBStream::UsbStreamPinConf_t usbPinConf =
    {
        .dataPlusPin_GPIObase = GPIOA, .dataPlusPin_number = 12, .dataPlusPin_alternate = 10,
        .dataMinusPin_GPIObase = GPIOA, .dataMinusPin_number = 11, .dataMinusPin_alternate = 10
    };
    USBStream::init(&usbPinConf, ASSERV_THREAD_FREQUENCY);

    chBSemSignal(&asservStarted_semaphore);

    mainAsserv->mainLoop();
}

void usbSerialCallback(char *buffer, uint32_t size);
static THD_WORKING_AREA(waLowPrioUSBThread, 1024);
static THD_FUNCTION(LowPrioUSBThread, arg)
{
    (void) arg;
    chRegSetThreadName("LowPrioUSBThread");


    while (!chThdShouldTerminateX())
    {
       USBStream::instance()->USBStreamHandleConnection_lowerpriothread(usbSerialCallback);
    }

}


#ifdef ENABLE_SHELL
THD_WORKING_AREA(wa_shell, 2048);
#else
THD_WORKING_AREA(wa_raspio1, 512);
THD_WORKING_AREA(wa_raspio2, 1024);
#endif

char history_buffer[SHELL_MAX_HIST_BUFF];
char *completion_buffer[SHELL_MAX_COMPLETIONS];

float config_buffer[35];
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

    outputStream = reinterpret_cast<BaseSequentialStream*>(&SD2);
    // chThdCreateStatic(waLowPrioUSBThread, sizeof(waLowPrioUSBThread), LOWPRIO, LowPrioUSBThread, NULL);


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

    
#ifdef ENABLE_SHELL
        thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, shellThread, &shellCfg);
        chRegSetThreadNameX(shellThd, "shell");
#else
    /* 
     *  Needed thread to run SerialIO (ie: here, communication with ESP32).
     *  C wrapping function are needed to bridge through C and C++
     */
        thread_t *threadPositionOutput = chThdCreateStatic(wa_raspio1, sizeof(wa_raspio1), LOWPRIO+1, serialIoWrapperPositionOutput, nullptr);
        chRegSetThreadNameX(threadPositionOutput, "positionOutput");
        thread_t *threadCommandInput = chThdCreateStatic(wa_raspio2, sizeof(wa_raspio2), LOWPRIO+2, serialIoWrapperCommandInput, nullptr);
        chRegSetThreadNameX(threadCommandInput, "commandInput");
#endif


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
        chprintf(outputStream," - asserv enablemotor 0|1\r\n");
        chprintf(outputStream," - asserv coders \r\n");
        chprintf(outputStream," - asserv reset \r\n");
        chprintf(outputStream," - asserv motorspeed [r|l] speed \r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv wheelspeedstep [r|l] [speed] [step time] \r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv robotfwspeedstep [speed] [step time] \r\n");
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
        chprintf(outputStream," - asserv wheelsdist distance_in_mm\r\n");
        chprintf(outputStream," -------------- \r\n");
        chprintf(outputStream," - asserv addgoto X Y\r\n");
        chprintf(outputStream," - asserv gototest\r\n");
    };
    (void) chp;

    if (argc == 0)
    {
        printUsage();
        return;
    }

    if (!strcmp(argv[0], "wheelspeedstep"))
    {
        char side = *argv[1];
        float speedGoal = atof(argv[2]);
        int time = atoi(argv[3]);
        chprintf(outputStream, "setting wheel %s to speed %.2f rad/s for %d ms \r\n", (side == 'r') ? "right" : "left", speedGoal, time);

        float speedRight = speedGoal;
        float speedLeft = 0;
        if (side == 'l')
        {
            speedLeft = speedGoal;
            speedRight = 0;
        }

        bool ok = commandManager->addWheelsSpeed(speedRight, speedLeft, time);
    }
    else if (!strcmp(argv[0], "robotfwspeedstep"))
    {
        float speedGoal = atof(argv[1]);
        int time = atoi(argv[2]);
        chprintf(outputStream, "setting fw robot speed %.2f rad/s for %d ms\r\n", speedGoal, time);

        bool ok = commandManager->addWheelsSpeed(speedGoal, speedGoal, time);
    }
    else if (!strcmp(argv[0], "speedcontrol"))
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

//        distanceAccelerationLimiter->setMaxAcceleration(acc_max);
//        distanceAccelerationLimiter->setMinAcceleration(acc_min);
//        distanceAccelerationLimiter->setHighSpeedThreshold(acc_threshold);
    }
    else if (!strcmp(argv[0], "distaccdec"))
    {
        float acc_fw = atof(argv[1]);
        float dec_fw = atof(argv[2]);
        float acc_bw = atof(argv[3]);
        float dec_bw = atof(argv[4]);
        float damp = atof(argv[5]);
        chprintf(outputStream, "setting distance acceleration dec limiter fw : acc%.2f dec%.2f bw: acc%.2f dec%.2f \r\n", acc_fw, dec_fw, acc_bw, dec_bw);

        distanceAccelerationLimiter->setMaxAccFW(acc_fw);
        distanceAccelerationLimiter->setMaxDecFW(dec_fw);
        distanceAccelerationLimiter->setMaxAccBW(acc_bw);
        distanceAccelerationLimiter->setMaxDecBW(dec_bw);
        distanceAccelerationLimiter->setDamplingFactor(damp);
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
        chprintf(outputStream, "Adding distance %.2fmm argc %d (%s)\r\n", dist,argc,  argv[1] );

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
    else if (!strcmp(argv[0], "wheelsdist"))
    {
        float wheelDistance_mm = atof(argv[1]);
        chprintf(outputStream, "setting wheels dist to %.2f \r\n", wheelDistance_mm);

		mainAsserv->setEncodersWheelsDistance_mm(wheelDistance_mm);
    }
    else if (!strcmp(argv[0], "enablemotor"))
    {
        bool enable = !(atoi(argv[1]) == 0);
        chprintf(outputStream, "%s motor output\r\n", (enable ? "enabling" : "disabling"));
        mainAsserv->enableMotors(enable);
    }
    else if (!strcmp(argv[0], "coders"))
    {
        chprintf(outputStream, "Encoders count %d %d \r\n", encoders->getRightEncoderTotalCount(), encoders->getLeftEncoderTotalCount());
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

        if (side == 'l')
            md22MotorController->setMotorLeftSpeed(speedGoal);
        else
            md22MotorController->setMotorRightSpeed(speedGoal);
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
        distanceAccelerationLimiter->setDamplingFactor(value);
    }
    else if (!strcmp(argv[0], "gototest"))
    {
        bool ok = commandManager->addStraightLine(1500);
        chprintf(outputStream, "Adding distance %.2fmm \r\n", 1500 );
        chThdSleepMilliseconds(600);
        chprintf(outputStream, "Stop!\r\n");

        mainAsserv->setEmergencyStop();

    }
    else if (!strcmp(argv[0], "get_config"))
    {
        uint8_t index = 0;

        // SpeedControllerLeft
        for( int i=0; i<NB_PI_SUBSET; i++)
        {
            speedControllerLeft->getGainsForRange(i, &config_buffer[index], &config_buffer[index+1], &config_buffer[index+2] );
            index += 3;
        }

        // SpeedControllerRight
        for( int i=0; i<NB_PI_SUBSET; i++)
        {
            speedControllerRight->getGainsForRange(i, &config_buffer[index], &config_buffer[index+1], &config_buffer[index+2]);
            index += 3;
        }

        //Regulators
        config_buffer[index++] = distanceRegulator->getGain();
        config_buffer[index++] = angleRegulator->getGain();

        // accel limiter
        config_buffer[index++] = angleAccelerationlimiter->getMaxAcceleration();
        config_buffer[index++] = distanceAccelerationLimiter->getMaxAccFW();
        config_buffer[index++] = distanceAccelerationLimiter->getMaxDecFW();
        config_buffer[index++] = distanceAccelerationLimiter->getMaxAccBW();
        config_buffer[index++] = distanceAccelerationLimiter->getMaxDecBW();
        config_buffer[index++] = distanceAccelerationLimiter->getDamplingFactor();
    

        chprintf(outputStream, "sending %d float of config !\r\n", index);
        USBStream::instance()->sendConfig((uint8_t*)config_buffer, index*sizeof(config_buffer[0]));
    }
    else
    {
        printUsage();
    }
}


void usbSerialCallback(char *buffer, uint32_t size)
{
    if (size > 0)
    {
        /*
         *  On transforme la commande recu dans une version argv/argc
         *    de manière a utiliser les commandes shell déjà définie...
         */
        bool prevWasSpace = false;
        char* firstArg = buffer;
        int nb_arg = 0;
        char *argv[10];
        for (uint32_t i = 0; i < size; i++)
        {
            if (prevWasSpace && buffer[i] != ' ')
            {
                argv[nb_arg++] = &buffer[i];
            }

            if (buffer[i] == ' ' || buffer[i] == '\r' || buffer[i] == '\n')
            {
                prevWasSpace = true;
                buffer[i] = 0;
            }
            else
            {
                prevWasSpace = false;
            }
        }

        // On évite de faire appel au shell si le nombre d'arg est mauvais ou si la 1ière commande est mauvaise...
        if (nb_arg > 0 && !strcmp(firstArg, "asserv"))
        {
            asservCommandUSB(nullptr, nb_arg, argv);
        }
    }
}
