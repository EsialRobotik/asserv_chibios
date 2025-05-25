#include "ch.h"
#include "hal.h"
#include "shell.h"
#include <chprintf.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cfloat>
#include "Encoders/QuadratureEncoder.h"
#include "motorController/Mp6550.h"
#include "sampleStream/USBStream.h"
#include "util/asservMath.h"
#include "util/chibiOsAllocatorWrapper.h"
#include "AsservMain.h"
#include "commandManager/CommandManager.h"
#include "SpeedController/SpeedController.h"
#include "SpeedController/AdaptativeSpeedController.h"
#include "Encoders/QuadratureEncoder.h"
#include "Odometry.h"
#include "AccelerationLimiter/SimpleAccelerationLimiter.h"
#include "AccelerationLimiter/AccelerationDecelerationLimiter.h"
#include "Pll.h"
#include "blockingDetector/OldSchoolBlockingDetector.h"
#include "config.h"
#include "Communication/SerialIO.h"

float speed_controller_right_Kp[NB_PI_SUBSET] = { SPEED_CTRL_RIGHT_KP_1, SPEED_CTRL_RIGHT_KP_2, SPEED_CTRL_RIGHT_KP_3};
float speed_controller_right_Ki[NB_PI_SUBSET] = { SPEED_CTRL_RIGHT_KI_1, SPEED_CTRL_RIGHT_KI_2, SPEED_CTRL_RIGHT_KI_3};
float speed_controller_right_SpeedRange[NB_PI_SUBSET] = { SPEED_CTRL_RIGHT_SPEED_THRES_1, SPEED_CTRL_RIGHT_SPEED_THRES_2, SPEED_CTRL_RIGHT_SPEED_THRES_3};

float speed_controller_left_Kp[NB_PI_SUBSET] = { SPEED_CTRL_LEFT_KP_1, SPEED_CTRL_LEFT_KP_2, SPEED_CTRL_LEFT_KP_3};
float speed_controller_left_Ki[NB_PI_SUBSET] = { SPEED_CTRL_LEFT_KI_1, SPEED_CTRL_LEFT_KI_2, SPEED_CTRL_LEFT_KI_3};
float speed_controller_left_SpeedRange[NB_PI_SUBSET] = { SPEED_CTRL_LEFT_SPEED_THRES_1, SPEED_CTRL_LEFT_SPEED_THRES_2, SPEED_CTRL_LEFT_SPEED_THRES_3};


Goto::GotoConfiguration preciseGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm};
Goto::GotoConfiguration waypointGotoConf  = {COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm, COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm};
GotoNoStop::GotoNoStopConfiguration gotoNoStopConf = {COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD, (50/DIST_REGULATOR_KP), 20};

Mp6550 *mp6550;
QuadratureEncoder *encoders;

Regulator *angleRegulator;
Regulator *distanceRegulator;

Odometry *odometry;

AdaptativeSpeedController *speedControllerRight;
AdaptativeSpeedController *speedControllerLeft;

Pll *rightPll;
Pll *leftPll;

SimpleAccelerationLimiter *angleAccelerationlimiter;

#ifdef STAR
AccelerationDecelerationLimiter *distanceAccelerationLimiter;
#else
SimpleAccelerationLimiter *distanceAccelerationLimiter;
#endif

CommandManager *commandManager;
AsservMain *mainAsserv;

SerialIO *esp32Io;

BaseSequentialStream *outputStream;



// // ADCConfig structure for stm32 MCUs is empty
// static ADCConfig adccfg = {};

// // Create buffer to store ADC results. This is
// // one-dimensional interleaved array
// #define ADC_BUF_DEPTH 2 // depth of buffer
// #define ADC_CH_NUM 1    // number of used ADC channels
// static adcsample_t samples_buf[ADC_BUF_DEPTH * ADC_CH_NUM]; // results array

// // Fill ADCConversionGroup structure fields
// static ADCConversionGroup adccg = {
//    // this 3 fields are common for all MCUs
//       // set to TRUE if need circular buffer, set FALSE otherwise
//       TRUE,
//       // number of channels
//       (uint16_t)(ADC_CH_NUM),
//       // callback function, set to NULL for begin
//       NULL,
//    // Resent fields are stm32 specific. They contain ADC control registers data.
//    // Please, refer to ST manual RM0008.pdf to understand what we do.
//       // CR1 register content, set to zero for begin
//       0,
//       // CR2 register content, set to zero for begin
//       0,
//       // SMRP1 register content, set to zero for begin
//       0,
//       // SMRP2 register content, set to zero for begin
//       0,
//       // SQR1 register content. Set channel sequence length
//       ADC_SQR1_NUM_CH(ADC_CH_NUM) | ADC_SQR1_SQ1_N(1),
//       // SQR2 register content, set to zero for begin
//       0,
//       // SQR3 register content. We must select 2 channels
//       // For example 15th and 10th channels. Refer to the
//       // pinout of your MCU to select other pins you need.
//       // On STM32-P103 board they connected to PC15 and PC0 contacts
//       // On STM32-103STK board they connected to EXT2-7 contact and joystick
//       0,
// };


static void initAsserv()
{

    Mp6550::Mp6550Conf_t mp6550Conf =
        {
            /*
            * MP6550 N°1 PWM 1
            * PWM TIM 1 chan 1 :
            *   pin : PA8
            *   alternate : 6
            * */
            .confPwm1Motor1 =
            {
                .pwm_config =
                {
                        2000000, /* 2Mhz PWM clock frequency.   */
                        100,     /*  PWM period @ 20khz = 50µs.       */
                        NULL,
                        {
                         {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                         {PWM_OUTPUT_DISABLED, NULL},
                         {PWM_OUTPUT_DISABLED, NULL},
                         {PWM_OUTPUT_DISABLED, NULL},
                        },
                         0,
                         0,
                         0
                },
                .pwmOutpin_GPIObase = GPIOA,
                .pwmOutpinNumber = 8,
                .pwmOutpin_alternate = 6,
                .used_channel = 0,
                .pwmDriver = &PWMD1
            },

            /*
            * MP6550 N°1 PWM 2
            * PWM TIM 8 chan 1 :
            *   pin : PA15
            *   alternate : 2
            * */
            .confPwm2Motor1 =
              {
                  .pwm_config =
                  {
                          2000000, /* 2Mhz PWM clock frequency.   */
                          100,     /*  PWM period @ 20khz = 50µs.       */
                          NULL,
                          {
                           {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                           {PWM_OUTPUT_DISABLED, NULL},
                           {PWM_OUTPUT_DISABLED, NULL},
                           {PWM_OUTPUT_DISABLED, NULL},
                          },
                           0,
                           0,
                           0
                  },
                  .pwmOutpin_GPIObase = GPIOA,
                  .pwmOutpinNumber = 15,
                  .pwmOutpin_alternate = 2,
                  .used_channel = 0,
                  .pwmDriver = &PWMD8
              },

              /*
              * MP6550 N°2 PWM 1
              * PWM TIM 4 chan 1 :
              *   pin : PB6
              *   alternate : 2
              * */
              .confPwm1Motor2 =
              {
                  .pwm_config =
                  {
                          2000000, /* 2Mhz PWM clock frequency.   */
                          100,     /*  PWM period @ 20khz = 50µs.       */
                          NULL,
                          {
                           {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                           {PWM_OUTPUT_DISABLED, NULL},
                           {PWM_OUTPUT_DISABLED, NULL},
                           {PWM_OUTPUT_DISABLED, NULL},
                          },
                           0,
                           0,
                           0
                  },
                  .pwmOutpin_GPIObase = GPIOB,
                  .pwmOutpinNumber = 6,
                  .pwmOutpin_alternate = 2,
                  .used_channel = 0,
                  .pwmDriver = &PWMD4
              },

              /*
             * MP6550 N°2 PWM 2
             * PWM TIM 17 chan 1 :
             *   pin : PA7
             *   alternate : 1
             * */
             .confPwm2Motor2 =
             {
                 .pwm_config =
                 {
                         2000000, /* 2Mhz PWM clock frequency.   */
                         100,     /*  PWM period @ 20khz = 50µs.       */
                         NULL,
                         {
                          {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                          {PWM_OUTPUT_DISABLED, NULL},
                          {PWM_OUTPUT_DISABLED, NULL},
                          {PWM_OUTPUT_DISABLED, NULL},
                         },
                          0,
                          0,
                          0
                 },
                 .pwmOutpin_GPIObase = GPIOA,
                 .pwmOutpinNumber = 7,
                 .pwmOutpin_alternate = 1,
                 .used_channel = 0,
                 .pwmDriver = &PWMD17
             }
        };

    /*
     *  MP6550 NO_sleep control
     *      Output Push/pull
     *      pin : PB5
     */
    Mp6550::sleepPin_t sleepPinConf =
    {
        .GPIObase = GPIOB,
        .pinNumber = 5
    };
    mp6550 = new Mp6550(mp6550Conf, &sleepPinConf, true, false, true);



    /**
     * TIM_3 chan 1/2 : PB4/PA4
     * TIM 2 chan 1/2 : PA5/PA1
     */
    QuadratureEncoder::GpioPinInit pami_encoders = {GPIOB, 4, GPIOA, 4, GPIOA, 5, GPIOA, 1};
    encoders = new QuadratureEncoder (&pami_encoders, true, false, true);

    angleRegulator = new Regulator(ANGLE_REGULATOR_KP, REGULATOR_MAX_SPEED_MM_PER_SEC);
    distanceRegulator = new Regulator(DIST_REGULATOR_KP, REGULATOR_MAX_SPEED_MM_PER_SEC);

    rightPll = new Pll (PLL_BANDWIDTH);
    leftPll = new Pll(PLL_BANDWIDTH);

    odometry = new Odometry (ENCODERS_WHEELS_DISTANCE_MM, 0, 0);

    speedControllerRight = new AdaptativeSpeedController(speed_controller_right_Kp, speed_controller_right_Ki, speed_controller_right_SpeedRange, 100, WHEELS_MAX_SPEED_MM_PER_SEC, ASSERV_THREAD_FREQUENCY);
    speedControllerLeft = new AdaptativeSpeedController(speed_controller_left_Kp, speed_controller_left_Ki, speed_controller_left_SpeedRange, 100, WHEELS_MAX_SPEED_MM_PER_SEC, ASSERV_THREAD_FREQUENCY);


    angleAccelerationlimiter = new SimpleAccelerationLimiter(ANGLE_REGULATOR_MAX_ACC);


    AccelerationDecelerationLimiter *accDecLimiter = nullptr;    
    #ifdef STAR
    distanceAccelerationLimiter = new AccelerationDecelerationLimiter(DIST_REGULATOR_MAX_ACC_FW, DIST_REGULATOR_MAX_DEC_FW, DIST_REGULATOR_MAX_ACC_BW, DIST_REGULATOR_MAX_DEC_BW, REGULATOR_MAX_SPEED_MM_PER_SEC, ACC_DEC_DAMPLING, DIST_REGULATOR_KP);
    accDecLimiter = distanceAccelerationLimiter;
    #else
    distanceAccelerationLimiter = new SimpleAccelerationLimiter(ANGLE_REGULATOR_MAX_ACC);
    #endif


    commandManager = new CommandManager( COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm, COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD,
                                   preciseGotoConf, waypointGotoConf, gotoNoStopConf,
                                   *angleRegulator, *distanceRegulator,
                                   REGULATOR_MAX_SPEED_MM_PER_SEC, REGULATOR_MAX_SPEED_MM_PER_SEC,
                                   accDecLimiter);

    mainAsserv = new AsservMain( ASSERV_THREAD_FREQUENCY, ASSERV_POSITION_DIVISOR,
                           ENCODERS_WHEELS_RADIUS_MM, ENCODERS_WHEELS_DISTANCE_MM, ENCODERS_TICKS_BY_TURN,
                           *commandManager, *mp6550, *encoders, *odometry,
                           *angleRegulator, *distanceRegulator,
                           *angleAccelerationlimiter, *distanceAccelerationLimiter,
                           *speedControllerRight, *speedControllerLeft,
                           *rightPll, *leftPll,
                           nullptr);

    esp32Io = new SerialIO(&SD1, *odometry, *commandManager, *mp6550, *mainAsserv);
}

void serialIoWrapperPositionOutput(void *)
{
    esp32Io->positionOutput();
}


void serialIoWrapperCommandInput(void *)
{
    esp32Io->commandInput();
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

    mp6550->init();
    encoders->init();
    encoders->start();
    USBStream::init(nullptr, ASSERV_THREAD_FREQUENCY);

    chBSemSignal(&asservStarted_semaphore);

    mainAsserv->mainLoop();
}

void usbSerialCallback(char *buffer, uint32_t size);
static THD_WORKING_AREA(waLowPrioUSBThread, 512);
static THD_FUNCTION(LowPrioUSBThread, arg)
{
    (void) arg;
    chRegSetThreadName("LowPrioUSBThread");


    while (!chThdShouldTerminateX())
    {
       USBStream::instance()->USBStreamHandleConnection_lowerpriothread(usbSerialCallback);
    }

}





THD_WORKING_AREA(wa_shell, 2048);
THD_WORKING_AREA(wa_controlPanel, 256);
//THD_FUNCTION(ControlPanelThread, p);

char history_buffer[SHELL_MAX_HIST_BUFF];
char *completion_buffer[SHELL_MAX_COMPLETIONS];

float config_buffer[30];
void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv);



THD_WORKING_AREA(wa_raspio1, 512);
THD_WORKING_AREA(wa_raspio2, 512);

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
    outputStream = reinterpret_cast<BaseSequentialStream*>(&LPSD1);
    shellInit();


    /*
     * Asserv Thread with sync mecanism
    */
    chBSemObjectInit(&asservStarted_semaphore, true);
    chThdCreateStatic(waAsservThread, sizeof(waAsservThread), HIGHPRIO, AsservThread, NULL);
    chBSemWait(&asservStarted_semaphore);


    /* Create a 'background' thread to handle command received through the USB */
    chThdCreateStatic(waLowPrioUSBThread, sizeof(waLowPrioUSBThread), LOWPRIO, LowPrioUSBThread, NULL);


    /*
     * Shell subSystem Init 
     */
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

    thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, shellThread, &shellCfg);
    chRegSetThreadNameX(shellThd, "shell");


//    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); // this is 15th channel
//    palSetPadMode(GPIOB, 7, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_FLOATING );
//    adcInit();
//  adcStart(&ADCD1, &adccfg);
//  adcStartConversion(&ADCD1, &adccg, &samples_buf[0], ADC_BUF_DEPTH);


    /* 
     *  Needed thread to run SerialIO (ie: here, communication with ESP32).
     *  C wrapping function are needed to bridge through C and C++
     */
    thread_t *threadPositionOutput = chThdCreateStatic(wa_raspio1, sizeof(wa_raspio1), LOWPRIO, serialIoWrapperPositionOutput, nullptr);
    chRegSetThreadNameX(threadPositionOutput, "positionOutput");
    thread_t *threadCommandInput = chThdCreateStatic(wa_raspio2, sizeof(wa_raspio2), LOWPRIO, serialIoWrapperCommandInput, nullptr);
    chRegSetThreadNameX(threadCommandInput, "commandInput");


    chThdSetPriority(LOWPRIO);
    
    while (true)
    {
        chThdSleepMilliseconds(1000);
    }
}

void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv)
{
    float encoderR, encoderL;
    auto printUsage = [](char * str)
    {
        chprintf(outputStream,"Unknown command : %s \r\n", str);
        chprintf(outputStream,"Usage :");
        chprintf(outputStream," - asserv wheelspeedstep [r|l] [speed] [step time ms]\r\n");
        chprintf(outputStream," - asserv robotfwspeedstep [speed] [step time ms] \r\n");
        chprintf(outputStream," - asserv orbital angleInDeg forward? toTheRight?\r\n");
        chprintf(outputStream," - asserv coders \r\n");
    };
    (void) chp;

    if (argc == 0)
    {
        printUsage("");
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
    else if (!strcmp(argv[0], "orbital"))
    {
        float angleInDeg = atof(argv[1]);
        bool forward = !(atoi(argv[2]) == 0);
        float turnToTheRight = atof(argv[3]);

        chprintf(outputStream, "Adding orbital turn of %.2f deg, forward? %d, to the right? %d \r\n", angleInDeg, forward, turnToTheRight);

        bool ok = commandManager->addGOrbitalTurn(angleInDeg*M_PI/180.0, forward, turnToTheRight);
    }
    else if (!strcmp(argv[0], "coders"))
    {
        encoders->getValues(&encoderR, &encoderL);
       chprintf(outputStream, "encoder %f %f (value vary only when mooving) \r\n", encoderR, encoderL);
    }
    else if (!strcmp(argv[0], "enablemotor"))
    {
        bool enable = !(atoi(argv[1]) == 0);
        chprintf(outputStream, "%s motor output\r\n", (enable ? "enabling" : "disabling"));
        mainAsserv->enableMotors(enable);
    }
    else if (!strcmp(argv[0], "adddist"))
    {
        float dist = atof(argv[1]);

        bool ok = commandManager->addStraightLine(dist);
        chprintf(outputStream, "Adding distance %.2fmm argc %d (%s)\r\n", dist,argc,  argv[1] );

    }
    else if (!strcmp(argv[0], "reset"))
    {
        mainAsserv->reset();
        chprintf(outputStream, "asserv resetted \r\n");
    }
    else if (!strcmp(argv[0], "enc"))
    {
        
        chprintf(outputStream, "Encodeurs count left %d \r\n", encoders->getLeftEncoderTotalCount());
        chprintf(outputStream, "Encodeurs count right %d \r\n",  encoders->getRightEncoderTotalCount());
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
    else if (!strcmp(argv[0], "addangle"))
    {
        float angle = atof(argv[1]);
        chprintf(outputStream, "Adding angle %.2frad \r\n", angle);

        commandManager->addTurn(angle);
    }
    else if (!strcmp(argv[0], "addgoto"))
    {
        float X = atof(argv[1]);
        float Y = atof(argv[2]);
        chprintf(outputStream, "Adding goto(%.2f,%.2f) consign\r\n", X, Y);

        commandManager->addGoTo(X, Y);
    }
    else if (!strcmp(argv[0], "gototest"))
    {
        commandManager->addGoTo(400, 0) ;
        commandManager->addGoTo(400, 100) ;
        commandManager->addGoTo(0, 100) ;
        commandManager->addGoTo(0, 0) ;
        commandManager->addGoToAngle(400, 0) ;

    }
    else
    {
        printUsage(argv[0]);
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
