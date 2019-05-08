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
#include "shell.h"
#include <chprintf.h>
#include <cstdlib>
#include <cstring>
#include "AsservMain.h"


AsservMain mainAsserv;
static THD_WORKING_AREA(waAsservThread, 512);
static THD_FUNCTION(AsservThread, arg)
{
	(void) arg;
	chRegSetThreadName("AsservThread");
	mainAsserv.init();
	mainAsserv.mainLoop();
}


THD_WORKING_AREA(wa_shell, 512);
void asservCommand(BaseSequentialStream *chp, int argc, char **argv);

BaseSequentialStream *outputStream;
int main(void)
{

	halInit();
	chSysInit();

	sdStart(&SD2, NULL);
	shellInit();

	outputStream =
			reinterpret_cast<BaseSequentialStream*>(&SD2);

	// Custom commands
	const ShellCommand shellCommands[] = {
	{
		"asserv",
		 &(asservCommand)
	},
	{nullptr, nullptr}
	};
	ShellConfig shellCfg =
	{
	/* sc_channel */outputStream,
	/* sc_commands */shellCommands,
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
//	motorController.setMotor1Speed(0);


//	chThdSetPriority(LOWPRIO);
	while (true)
	{
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
		chprintf(outputStream," - asserv setspeed [r|l] [speed]\r\n");
		chprintf(outputStream," - asserv speedstep [speed] [step time] \r\n");
		chprintf(outputStream," - asserv speedcontrol [Kp] [Ki] \r\n");

	};
	(void) chp;

	if (argc == 0) {
		printUsage();
		return;
	}

	if(!strcmp(argv[0], "setspeed"))
	{
		char side = *argv[1];
		float   speedGoal = atof(argv[2]);
		float speedGoalRight=0;
		float speedGoalLeft=0;

		if( side == 'r')
			speedGoalRight = speedGoal;
		else if( side == 'l')
			speedGoalLeft = speedGoal;


		chprintf(outputStream, "setting speed left %f right %f rad/s \r\n",speedGoalLeft,speedGoalRight);
		mainAsserv.setMotorsSpeed(speedGoalLeft, speedGoalRight);
	}
	else if(!strcmp(argv[0], "speedstep"))
	{
		float   speedGoal = atof(argv[1]);
		int time = atoi(argv[2]);
		chprintf(outputStream, "setting speed %f rad/s for %d ms \r\n",speedGoal,time);
		mainAsserv.setMotorsSpeed(speedGoal, speedGoal);
		chThdSleepMilliseconds(time);
		mainAsserv.setMotorsSpeed(0.0, 0.0);
	}
	else if(!strcmp(argv[0], "speedcontrol"))
	{
		float   Kp = atof(argv[1]);
		float   Ki = atof(argv[2]);

		chprintf(outputStream, "setting speed control Kp:%f Ki:%f \r\n",Kp,Ki);
		mainAsserv.setGainForLeftSpeedController(Kp, Ki);
		mainAsserv.setGainForRightSpeedController(Kp, Ki);

	}
	else
	{
		printUsage();
	}

}
