/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "rt_test_root.h"
#include "oslib_test_root.h"
#include "shell.h"
#include <chprintf.h>
#include "USBStream.hpp"

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(1000);
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(1000);
  }
}


THD_WORKING_AREA(wa_shell, 512);


static PWMConfig pwmcfg = {
   200000,
   1000,
   NULL,
   {
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
   },
   0,
   0
};

static PWMConfig pwmcfg2 = {
   200000,
   1000,
   NULL,
   {
	  {PWM_OUTPUT_ACTIVE_HIGH, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL},
      {PWM_OUTPUT_DISABLED, NULL}
   },
   0,
   0
};



QEIConfig qeicfg1 = {
  QEI_MODE_QUADRATURE,
  QEI_BOTH_EDGES,
  QEI_DIRINV_FALSE,
};

QEIConfig qeicfg2 = {
  QEI_MODE_QUADRATURE,
  QEI_BOTH_EDGES,
  QEI_DIRINV_FALSE,
};


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();


  // MOTOR 1 control
  palSetPadMode(GPIOA, 10, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOA, 10); //M1INA
  palSetPad(GPIOB, 5); //M1INB

  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(3)); //TIM4_chan1
  pwmStart(&PWMD8, &pwmcfg);
  pwmEnableChannel(&PWMD8, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1000));   // 90% duty cycle

  // MOTOR 2 control
  palSetPadMode(GPIOA, 8, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOA, 8); //M2INA
  palClearPad(GPIOA, 9);//M2INB

  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(2)); //TIM4_chan1
  pwmStart(&PWMD4, &pwmcfg2);
  pwmEnableChannel(&PWMD4,0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 1000));


  // Encoder 1
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(2)); //TIM3_chan2
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(2)); //TIM3_chan1
  qeiStart(&QEID3, &qeicfg1);
  qeiEnable(&QEID3);

  // Encoder 2
  palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(1)); //TIM2_chan1
  palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(1)); //TIM2_chan2
  qeiStart(&QEID2, &qeicfg2);
  qeiEnable(&QEID2);

  // USB FS
  palSetPadMode(GPIOA, 12, PAL_MODE_ALTERNATE(10)); //USB D+
  palSetPadMode(GPIOA, 11, PAL_MODE_ALTERNATE(10)); //USB D-
  USBStream::init();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);



	/*
	 * Activates the serial driver 2 using the driver default configuration.
	 */
	sdStart(&SD2, NULL);
	shellInit();

	BaseSequentialStream *outputStream = reinterpret_cast<BaseSequentialStream*>(&SD2);

	ShellConfig shellCfg = {
			/* sc_channel */    outputStream,
			/* sc_commands */   NULL,
	#if (SHELL_USE_HISTORY == TRUE)
			/* sc_histbuf */    histbuf,
			/* sc_histsize */   sizeof(histbuf),
	#endif
	#if (SHELL_USE_COMPLETION == TRUE)
			/* sc_completion */ completion_buffer
	#endif
		};

	thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, shellThread, &shellCfg);
	chRegSetThreadNameX(shellThd, "shell");




  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
	  int16_t qei2  = qeiGetCount(&QEID2);
	  int16_t qei3  = qeiGetCount(&QEID3);
	  qei_lld_set_count(&QEID2, 0);
	  qei_lld_set_count(&QEID3, 0);


  void *ptr = USBStream::instance()->SendCurrentStream();

  chprintf(outputStream, "qei2 %d quei3 %d  ptr %x \r\n",  qei2, qei3, ptr);

    chThdSleepMilliseconds(750);
  }
}
