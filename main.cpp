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
#include "Vnh5019.h"

#define ASSERV_THREAD_PERIOD_MS 2
static THD_WORKING_AREA(waAsservThread, 512);
static THD_FUNCTION(AsservThread, arg)
{
	(void) arg;
	chRegSetThreadName("AsservThread");

	systime_t time = chVTGetSystemTime();
	time += TIME_MS2I(ASSERV_THREAD_PERIOD_MS);
	while (true)
	{

		chThdSleepUntil(time);
		time += TIME_MS2I(ASSERV_THREAD_PERIOD_MS);
	}
}


static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(500);
  }
}


THD_WORKING_AREA(wa_shell, 512);



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


int main(void)
{

	halInit();
	chSysInit();

	Vnh5019 motorController;
	motorController.init();

	// Encoder 1
	palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(2)); //TIM3_chan2
	palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(2)); //TIM3_chan1
	qeiStart(&QEID3, &qeicfg1);
	qeiEnable (&QEID3);

	// Encoder 2
	palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(1)); //TIM2_chan1
	palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(1)); //TIM2_chan2
	qeiStart(&QEID2, &qeicfg2);
	qeiEnable (&QEID2);

	// USB FS
	palSetPadMode(GPIOA, 12, PAL_MODE_ALTERNATE(10)); //USB D+
	palSetPadMode(GPIOA, 11, PAL_MODE_ALTERNATE(10)); //USB D-
	USBStream::init();

	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

	sdStart(&SD2, NULL);
	shellInit();

	BaseSequentialStream *outputStream =
			reinterpret_cast<BaseSequentialStream*>(&SD2);

	ShellConfig shellCfg =
	{
	/* sc_channel */outputStream,
	/* sc_commands */NULL,
#if (SHELL_USE_HISTORY == TRUE)
			/* sc_histbuf */histbuf,
			/* sc_histsize */sizeof(histbuf),
#endif
#if (SHELL_USE_COMPLETION == TRUE)
			/* sc_completion */completion_buffer
#endif
		};

	thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO,
			shellThread, &shellCfg);
	chRegSetThreadNameX(shellThd, "shell");

	chThdCreateStatic(waAsservThread, sizeof(waAsservThread), HIGHPRIO, AsservThread, NULL);


//	motorController.setMotor1Speed(10);
//	motorController.setMotor2Speed(-10);
	motorController.setMotor1Speed(0);



	while (true)
	{
		int16_t qei2 = qeiGetCount(&QEID2);
		int16_t qei3 = qeiGetCount(&QEID3);
		qei_lld_set_count(&QEID2, 0);
		qei_lld_set_count(&QEID3, 0);

		void *ptr = USBStream::instance()->SendCurrentStream();

//  chprintf(outputStream, "qei2 %d quei3 %d  ptr %x \r\n",  qei2, qei3, ptr);

		chThdSleepMilliseconds(750);
	}
}
