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


#define ENCODERS_WHEELS_RADIUS (47.2/2.0)

AsservMain mainAsserv(ENCODERS_WHEELS_RADIUS);
static THD_WORKING_AREA(waAsservThread, 512);
static THD_FUNCTION(AsservThread, arg)
{
	(void) arg;
	chRegSetThreadName("AsservThread");
	mainAsserv.init();
	mainAsserv.mainLoop();
}


THD_WORKING_AREA(wa_shell, 512);

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
			/* sc_histbuf */history_buffer,
			/* sc_histsize */sizeof(history_buffer),
#endif
#if (SHELL_USE_COMPLETION == TRUE)
			/* sc_completion */completion_buffer
#endif
		};

	thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO,
			shellThread, &shellCfg);
	chRegSetThreadNameX(shellThd, "shell");

	chThdCreateStatic(waAsservThread, sizeof(waAsservThread), HIGHPRIO, AsservThread, NULL);



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
		chprintf(outputStream," - asserv speedstep [r|l] [speed] [step time] \r\n");
		chprintf(outputStream," - asserv speedstep2 [speed] [step time] \r\n");
		chprintf(outputStream," - asserv speedcontrol [r|l] [Kp] [Ki] \r\n");
		chprintf(outputStream," - asserv speedslope [r|l] delta_speed \r\n");


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

		if( side == 'r') speedGoalRight = speedGoal;
		else if( side == 'l') speedGoalLeft = speedGoal;


		chprintf(outputStream, "setting speed left %.2f right %.2f rad/s \r\n",speedGoalLeft,speedGoalRight);
		mainAsserv.setMotorsSpeed(speedGoalLeft, speedGoalRight);
	}
	else if(!strcmp(argv[0], "speedstep"))
	{
		char side = *argv[1];
		float   speedGoal = atof(argv[2]);
		float speedGoalRight = 0, speedGoalLeft = 0;
		int time = atoi(argv[3]);
		chprintf(outputStream, "setting speed %.2f rad/s for %d ms for side %c \r\n",speedGoal,time, side);

		if( side == 'r') speedGoalRight = speedGoal;
		else if( side == 'l') speedGoalLeft = speedGoal;

		mainAsserv.setMotorsSpeed(speedGoalLeft, speedGoalRight);
		chThdSleepMilliseconds(time);
		mainAsserv.setMotorsSpeed(0.0, 0.0);
	}
	else if(!strcmp(argv[0], "speedstep2"))
	{
		float   speedGoal = atof(argv[1]);
		int time = atoi(argv[2]);
		chprintf(outputStream, "setting speed %.2f rad/s for %d ms for both side \r\n",speedGoal,time);

		mainAsserv.setMotorsSpeed(speedGoal, speedGoal);
		chThdSleepMilliseconds(time);
		mainAsserv.setMotorsSpeed(0.0, 0.0);
	}
	else if(!strcmp(argv[0], "speedcontrol"))
	{
		char side = *argv[1];
		float   Kp = atof(argv[2]);
		float   Ki = atof(argv[3]);

		chprintf(outputStream, "setting speed control Kp:%.2f Ki:%.2f to side %c \r\n",Kp,Ki, side);

		if( side == 'r')
			mainAsserv.setGainForRightSpeedController(Kp, Ki);
		else if( side == 'l')
			mainAsserv.setGainForLeftSpeedController(Kp, Ki);
	}
	else if(!strcmp(argv[0], "speedslope"))
	{
		float   slope = atof(argv[1]);
		chprintf(outputStream, "setting speed slope delta %.2f \r\n",slope);

		mainAsserv.setSpeedSlope(slope);
	}
	else
	{
		printUsage();
	}

}
