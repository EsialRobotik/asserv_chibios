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






THD_WORKING_AREA(wa_shell, 2048);
THD_WORKING_AREA(wa_controlPanel, 256);
//THD_FUNCTION(ControlPanelThread, p);

char history_buffer[SHELL_MAX_HIST_BUFF];
char *completion_buffer[SHELL_MAX_COMPLETIONS];

float config_buffer[30];
void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv);



BaseSequentialStream *outputStream;

Mp6550 *mp6550;

int main(void)
{
    halInit();
    chSysInit();


    shellInit();

    sdStart(&LPSD1, NULL);
    outputStream = reinterpret_cast<BaseSequentialStream*>(&LPSD1);



    /*
     * USART 1:
     * RX: PA10
     * TX: PA9
     */
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
    sdStart(&SD1, NULL);



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

    mp6550 = new Mp6550(mp6550Conf, true, false, false);
    mp6550->init();

//
//    /*
//     * MP6550 N°1
//     * PWM TIM 1 chan 1 :  PA8
//     * PWM TIM 8 chan 1 :  PA15
//     * ADC 1 in 1 : PA0
//     * */
//
//    static PWMConfig pwmcfg_tim1 = {
//      2000000,                                    /* 2Mhz PWM clock frequency.   */
//      100,                                    /*  PWM period @ 20khz = 50µs.       */
//      NULL,
//      {
//       {PWM_OUTPUT_ACTIVE_HIGH, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//      },
//       0,
//       0,
//       0
//    };
//    static PWMConfig pwmcfg_tim8 = {
//      2000000,                                    /* 2Mhz PWM clock frequency.   */
//      100,                                    /*  PWM period @ 20khz = 50µs.       */
//      NULL,
//      {
//       {PWM_OUTPUT_ACTIVE_HIGH, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//      },
//       0,
//       0,
//       0
//    };
//
//    pwmStart(&PWMD1, &pwmcfg_tim1);
//    pwmStart(&PWMD8, &pwmcfg_tim8);
//    pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 8000));
//    pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 10000));
//    palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(6)); //TIM1_chan1
//    palSetPadMode(GPIOA, 15, PAL_MODE_ALTERNATE(2)); //TIM8_chan1
//
//    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG); //GPIO current measure
////    adcStart(&ADCD1, NULL);
////    const ADCConversionGroup adcgrpcfg1 = {
////      false,
////      1,
////      NULL,
////      NULL,
////      0,                                                    /* CR1   */
////      ADC_CR2_SWSTART,        /* CR2   */
////      ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_144) |
////      ADC_SMPR1_SMP_VREF(ADC_SAMPLE_144),                   /* SMPR1 */
////      0,                                                    /* SMPR2 */
////      0,                                                    /* HTR */
////      0,                                                    /* LTR */
////      0,                                                    /* SQR1  */
////      0,                                                    /* SQR2  */
////      ADC_SQR3_SQ2_N(ADC_CHANNEL_SENSOR) |
////      ADC_SQR3_SQ1_N(ADC_CHANNEL_VREFINT)                   /* SQR3  */
////    };
////    adcsample_t samples[5];
////    adcConvert(&ADCD1, &adcgrpcfg1, samples, 5 );
////    palReadLine( PAL_LINE(GPIOA, 9U));
//
//
//    /*
//     * MP6550 N°1
//     * PWM TIM 4 chan 1 :  PB6
//     * PWM TIM 17 chan 1 :  PA7
//     * ADC 2 in 3 : PA6
//     * */
//
//    static PWMConfig pwmcfg_tim4 = {
//    2000000,                                    /* 2Mhz PWM clock frequency.   */
//    100,                                    /*  PWM period @ 20khz = 50µs.      */
//      NULL,
//      {
//       {PWM_OUTPUT_ACTIVE_HIGH, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//      },
//       0,
//       0,
//       0
//    };
//
//    static PWMConfig pwmcfg_tim17 = {
//    2000000,                                    /* 2Mhz PWM clock frequency.   */
//    100,                                    /*  PWM period @ 20khz = 50µs.      */
//      NULL,
//      {
//       {PWM_OUTPUT_ACTIVE_HIGH, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//       {PWM_OUTPUT_DISABLED, NULL},
//      },
//       0,
//       0,
//       0
//    };
//
//    pwmStart(&PWMD4, &pwmcfg_tim4);
//    pwmStart(&PWMD17, &pwmcfg_tim17);
//
//    pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4,  10000));
//    pwmEnableChannel(&PWMD17, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD17, 2000));
//    palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(2)); //TIM4_chan1
//    palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(1)); //TIM17_chan1
//
//    palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG); //GPIO current measure
////    palReadLine( PAL_LINE(GPIOA, 10U));



    /**
     * TIM_3 chan 1/2 : PA6/PA4
     * TIM 2 chan 1/2 : PA0/PA1
     */
    QuadratureEncoder::GpioPinInit pami_encoders = {GPIOA, 6, GPIOA, 4, GPIOA, 0, GPIOA, 1};
    QuadratureEncoder encoder(&pami_encoders, true);
    encoder.init();
    encoder.start();




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

    thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, shellThread, &shellCfg);
    chRegSetThreadNameX(shellThd, "shell");

    // Le thread controlPanel n'a de sens que quand le shell tourne
//        thread_t *controlPanelThd = chThdCreateStatic(wa_controlPanel, sizeof(wa_controlPanel), LOWPRIO, ControlPanelThread, nullptr);
//        chRegSetThreadNameX(controlPanelThd, "controlPanel");

    chThdSetPriority(LOWPRIO);
    float encoderL;
    float encoderR;
    while (true)
    {
        palClearPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(250);
        palSetPad(GPIOA, GPIOA_LED_GREEN);
        chThdSleepMilliseconds(250);
        encoder.getValues(&encoderR, &encoderL);
//        chprintf(outputStream, "encoder %f %f\r\n", encoderR, encoderL);
    }
}

void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv)
{
    auto printUsage = []()
    {
        chprintf(outputStream,"Usage :");
        chprintf(outputStream," - asserv motorspeed [r|l] speed \r\n");
    };
    (void) chp;

    if (argc == 0)
    {
        printUsage();
        return;
    }

    if (!strcmp(argv[0], "motorspeed"))
    {
        char side = *argv[1];
        float speedGoal = atof(argv[2]);

        chprintf(outputStream, "setting wheel %s to speed %.2f \r\n", (side == 'r') ? "right" : "left", speedGoal);

        if (side == 'l')
            mp6550->setMotorLeftSpeed(speedGoal);
        else
            mp6550->setMotorRightSpeed(speedGoal);
    }
    else
    {
        printUsage();
    }
}

