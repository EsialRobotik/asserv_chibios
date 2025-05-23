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
#include "Encoders/QuadratureEncoder.h"
#include "motorController/Md22.h"
#include "Odometry.h"
#include "AccelerationLimiter/SimpleAccelerationLimiter.h"
#include "AccelerationLimiter/AdvancedAccelerationLimiter.h"
#include "AccelerationLimiter/AccelerationDecelerationLimiter.h"
#include "Pll.h"
#include "blockingDetector/OldSchoolBlockingDetector.h"
#include "config.h"
#include "Encoders/MagEncoders.h"
#include "util/debug.h"


////float speed_controller_right_Kp[NB_PI_SUBSET] = { 0.3, 0.2, 0.1 };
////float speed_controller_right_Ki[NB_PI_SUBSET] = { 3.0, 4.2, 1.5 };
////float speed_controller_right_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60 };
////
////float speed_controller_left_Kp[NB_PI_SUBSET] = { 0.3, 0.2, 0.1 }; //0.08
////float speed_controller_left_Ki[NB_PI_SUBSET] = { 3.0, 4.2, 1.5 }; //1.0
////float speed_controller_left_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60 };

float speed_controller_right_Kp[NB_PI_SUBSET] = { 0.1, 0.1, 0.1 };
float speed_controller_right_Ki[NB_PI_SUBSET] = { 1.0, 0.8, 0.6 };
float speed_controller_right_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60 };

float speed_controller_left_Kp[NB_PI_SUBSET] = { 0.1, 0.1, 0.1 };
float speed_controller_left_Ki[NB_PI_SUBSET] = { 1.0, 0.8, 0.6 };
float speed_controller_left_SpeedRange[NB_PI_SUBSET] = { 20, 50, 60 };

Goto::GotoConfiguration preciseGotoConf = { COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm,
COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_PRECISE_ARRIVAL_DISTANCE_mm };

//#define COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm (20)
Goto::GotoConfiguration waypointGotoConf = { COMMAND_MANAGER_GOTO_RETURN_THRESHOLD_mm,
COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD, COMMAND_MANAGER_GOTO_WAYPOINT_ARRIVAL_DISTANCE_mm };

//#define COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD (M_PI/2)
GotoNoStop::GotoNoStopConfiguration gotoNoStopConf = { COMMAND_MANAGER_GOTO_ANGLE_THRESHOLD_RAD,
COMMAND_MANAGER_GOTONOSTOP_TOO_BIG_ANGLE_THRESHOLD_RAD, (100 / DIST_REGULATOR_KP), 85 };

Md22::I2cPinInit md22PMXCardPinConf_SCL_SDA = { GPIOB, 6, GPIOB, 7 };
QuadratureEncoder::GpioPinInit qePMXCardPinConf_E1ch1_E1ch2_E2ch1_E2ch2 = { GPIOC, 6, GPIOA, 7, GPIOA, 5, GPIOB, 9 };
MagEncoders::I2cPinInit encodersI2cPinsConf_SCL_SDA = { GPIOB, 10, GPIOB, 3 };

QuadratureEncoder *encoders;
MagEncoders *encoders_ext;
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
AdvancedAccelerationLimiter *distanceAccelerationLimiterAdv;
AccelerationDecelerationLimiter *distanceAccelerationLimiter;

CommandManager *commandManager;
AsservMain *mainAsserv;

BaseSequentialStream *outputStream;
BaseSequentialStream *outputStreamSd4;

static void initAsserv()
{
	debug1("initAsserv()...\r\n");

	//LED CLEAR
	palClearPad(GPIOA, GPIOA_ARD_D8);
	palClearPad(GPIOA, GPIOA_ARD_D12);

	md22MotorController = new Md22(&md22PMXCardPinConf_SCL_SDA, false, false, false, 400000); //400k
	debug1("initAsserv::md22MotorController OK\r\n");
	encoders = new QuadratureEncoder(&qePMXCardPinConf_E1ch1_E1ch2_E2ch1_E2ch2, false, true, false);
	debug1("initAsserv::QuadratureEncoder OK\r\n");
	encoders_ext = new MagEncoders(&encodersI2cPinsConf_SCL_SDA, false, false, true, 400000);
	debug1("initAsserv::MagEncoders OK\r\n");

	angleRegulator = new Regulator(ANGLE_REGULATOR_KP, REGULATOR_MAX_SPEED_MM_PER_SEC);
	distanceRegulator = new Regulator(DIST_REGULATOR_KP, FLT_MAX);

	rightPll = new Pll(PLL_BANDWIDTH);
	leftPll = new Pll(PLL_BANDWIDTH);

	odometry = new Odometry(ENCODERS_WHEELS_DISTANCE_MM, 0, 0);

	speedControllerRight = new AdaptativeSpeedController(speed_controller_right_Kp, speed_controller_right_Ki,
			speed_controller_right_SpeedRange, 100, WHEELS_MAX_SPEED_MM_PER_SEC, ASSERV_THREAD_FREQUENCY);
	speedControllerLeft = new AdaptativeSpeedController(speed_controller_left_Kp, speed_controller_left_Ki,
			speed_controller_left_SpeedRange, 100, WHEELS_MAX_SPEED_MM_PER_SEC, ASSERV_THREAD_FREQUENCY);

	angleAccelerationlimiter = new SimpleAccelerationLimiter(ANGLE_REGULATOR_MAX_ACC);
	distanceAccelerationLimiterAdv = new AdvancedAccelerationLimiter(DIST_REGULATOR_MAX_ACC, DIST_REGULATOR_MIN_ACC, DIST_REGULATOR_HIGH_SPEED_THRESHOLD);
//	distanceAccelerationLimiter = new AccelerationDecelerationLimiter(DIST_REGULATOR_MAX_ACC_FW,
//			DIST_REGULATOR_MAX_DEC_FW, DIST_REGULATOR_MAX_ACC_BW, DIST_REGULATOR_MAX_DEC_BW,
//			REGULATOR_MAX_SPEED_MM_PER_SEC, ACC_DEC_DAMPLING, DIST_REGULATOR_KP);

	blockingDetector = new OldSchoolBlockingDetector(ASSERV_THREAD_PERIOD_S, *md22MotorController, *odometry,
			BLOCKING_DETECTOR_ANGLE_SPEED_THRESHOLD_RAD_PER_S, BLOCKING_DETECTOR_DIST_SPEED_THRESHOLD_MM_PER_S,
			BLOCKING_DETECTOR_BLOCKING_DURATION_THRESHOLD_SEC, BLOCKING_DETECTOR_MINIMUM_CONSIDERED_SPEED_PERCENT);

	debug1("initAsserv::blockingDetector OK\r\n");

	commandManager = new CommandManager( COMMAND_MANAGER_ARRIVAL_DISTANCE_THRESHOLD_mm,
	COMMAND_MANAGER_ARRIVAL_ANGLE_THRESHOLD_RAD, preciseGotoConf, waypointGotoConf, gotoNoStopConf, *angleRegulator,
			*distanceRegulator, REGULATOR_MAX_SPEED_MM_PER_SEC,
			REGULATOR_MAX_SPEED_MM_PER_SEC / 3 /* This value should maybe be fine tuned ?*/,
			nullptr, blockingDetector); //distanceAccelerationLimiter
	debug1("initAsserv::commandManager OK\r\n");

	mainAsserv = new AsservMain( ASSERV_THREAD_FREQUENCY, ASSERV_POSITION_DIVISOR,
	ENCODERS_WHEELS_RADIUS_MM, ENCODERS_WHEELS_DISTANCE_MM, ENCODERS_TICKS_BY_TURN, *commandManager,
			*md22MotorController, *encoders, *odometry, *angleRegulator, *distanceRegulator, *angleAccelerationlimiter,
			*distanceAccelerationLimiterAdv, *speedControllerRight, *speedControllerLeft, *rightPll, *leftPll,
			blockingDetector);

	//debug1("initAsserv::mainAsserv OK\r\n");

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
	debug1("AsservThread()...\r\n");

	md22MotorController->init();
	debug1("AsservThread::md22MotorController OK\r\n");

	encoders->init();
	debug1("AsservThread::encoders init OK\r\n");
	encoders->start();
	debug1("AsservThread::encoders start OK\r\n");

	encoders_ext->init();
	debug1("AsservThread::encodersEXT start OK\r\n");
	encoders_ext->start();
	debug1("AsservThread::encodersEXT start OK\r\n");

	USBStream::UsbStreamPinConf_t usbPinConf = { .dataPlusPin_GPIObase = GPIOA, .dataPlusPin_number = 12,
			.dataPlusPin_alternate = 10, .dataMinusPin_GPIObase = GPIOA, .dataMinusPin_number = 11,
			.dataMinusPin_alternate = 10 };
	USBStream::init(&usbPinConf);

	debug1("AsservThread::USBStream init OK + chBSemSignal\r\n");

	chBSemSignal(&asservStarted_semaphore);

	//desactivation au demarrage
	mainAsserv->enableMotors(false);
	//debug1("AsservThread::enableMotors false\r\n");

	mainAsserv->mainLoop();

//    while (true)
//    {
//        palClearPad(GPIOA, GPIOA_ARD_D12);
//        chThdSleepMilliseconds(200);
//        palSetPad(GPIOA, GPIOA_ARD_D12);
//        chThdSleepMilliseconds(200);
//    }
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
THD_WORKING_AREA(wa_shell_serie, 2048);
THD_WORKING_AREA(wa_controlPanel_serie, 256);
//THD_FUNCTION(ControlPanelThread, p);

char history_buffer[SHELL_MAX_HIST_BUFF];
char *completion_buffer[SHELL_MAX_COMPLETIONS];

float config_buffer[30];
void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv);

void asservCommandSerial();

int main(void)
{
	halInit();
	chSysInit();
	//Config des PINs pour LEDs
	palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOA, 9, PAL_MODE_OUTPUT_PUSHPULL);
	//LED CLEAR
//        palClearPad(GPIOA, GPIOA_ARD_D8);
//        palClearPad(GPIOA, GPIOA_ARD_D12);
	palSetPad(GPIOA, GPIOA_ARD_D8);
	palSetPad(GPIOA, GPIOA_ARD_D12);

	//init de l'USB debug + SHELL
	sdStart(&SD2, NULL);
	outputStream = reinterpret_cast<BaseSequentialStream*>(&SD2);
	debug1("\r\nmain::STARTING SD2...\r\n");

	//LED CLEAR
	//        palClearPad(GPIOA, GPIOA_ARD_D8);
	//        palClearPad(GPIOA, GPIOA_ARD_D12);
	//LED GO
	palSetPad(GPIOA, GPIOA_ARD_D8);
	palSetPad(GPIOA, GPIOA_ARD_D12);
	chThdSleepMilliseconds(500);
//    while (true)
//        {
//            palClearPad(GPIOA, GPIOA_ARD_D8);
//            palClearPad(GPIOA, GPIOA_ARD_D12);
//            chThdSleepMilliseconds(500);
//            palSetPad(GPIOA, GPIOA_ARD_D8);
//            palSetPad(GPIOA, GPIOA_ARD_D12);
//            chThdSleepMilliseconds(500);
//
//            debug1("blinking ...\r\n");
//        }

#if DEBUG_PRINT == 1
    chprintf(outputStream, "Start OK SD2 STM32_PCLK1=%d STM32_SYSCLK=%d STM32_PLLCLKOUT=%d \r\n", STM32_PCLK1,
            STM32_SYSCLK, STM32_PLLCLKOUT);
    chprintf(outputStream, "STM32_PLLVCO=%d / STM32_PLLP_VALUE=%d \r\n", STM32_PLLVCO, STM32_PLLP_VALUE);
    chprintf(outputStream, "STM32_PLLCLKIN=%d * STM32_PLLN_VALUE=%d\r\n", STM32_PLLCLKIN, STM32_PLLN_VALUE);
#endif

	//config UART4 for raspIO
	palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(8));
	palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(8));
	sdStart(&SD4, NULL);
	outputStreamSd4 = reinterpret_cast<BaseSequentialStream*>(&SD4);
	chprintf(outputStreamSd4, "main::SD4 OK\r\n");

	//creation de tous les objets
	initAsserv();

	//debug1("main::initAsserv END.\r\n");

	chBSemObjectInit(&asservStarted_semaphore, true);
	//debug1("main::chBSemWait\r\n");
	chThdCreateStatic(waAsservThread, sizeof(waAsservThread), HIGHPRIO, AsservThread, NULL);
	chBSemWait(&asservStarted_semaphore);
	//debug1("main::waAsservThread END.\r\n");

	chThdCreateStatic(waLowPrioUSBThread, sizeof(waLowPrioUSBThread), LOWPRIO, LowPrioUSBThread, NULL);

	shellInit();
	//debug1("main::shellInit END.\r\n");

//    while (true)
//                            {
//                                palClearPad(GPIOA, GPIOA_ARD_D8);
//                                //palClearPad(GPIOA, GPIOA_ARD_D12);
//                                chThdSleepMilliseconds(1000);
//                                palSetPad(GPIOA, GPIOA_ARD_D8);
//                                //palSetPad(GPIOA, GPIOA_ARD_D12);
//                                chThdSleepMilliseconds(1000);
//
//                                debug1("blinking ...\r\n");
//                            }

	// Custom commands
	const ShellCommand shellCommands[] = { { "asserv", &(asservCommandUSB) }, { nullptr, nullptr } };
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

//#ifdef ENABLE_SHELL
//	bool startShell = true;
//#else
//    bool startShell = false;
//#endif
//	if (startShell)
//	{
//		//debug1("main::startShell ...\r\n");
//
//		thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, shellThread, &shellCfg);
//		chRegSetThreadNameX(shellThd, "shell");
//
//		// Le thread controlPanel n'a de sens que quand le shell tourne
//		thread_t *controlPanelThd = chThdCreateStatic(wa_controlPanel, sizeof(wa_controlPanel), LOWPRIO,
//				ControlPanelThread, nullptr);
//		chRegSetThreadNameX(controlPanelThd, "controlPanel");
//
//		thread_t *asserCmdSerialThread = chThdCreateStatic(wa_shell_serie, sizeof(wa_shell_serie), LOWPRIO,
//				asservCommandSerial, nullptr);
//		chRegSetThreadNameX(asserCmdSerialThread, "asserv Command serial");
//
//		thread_t *controlPanelThdSerial = chThdCreateStatic(wa_controlPanel_serie, sizeof(wa_controlPanel_serie),
//		LOWPRIO, asservPositionSerial, nullptr);
//		chRegSetThreadNameX(controlPanelThdSerial, "asserv position update serial");
//
//	}
//
//	deactivateHeapAllocation();
#ifdef ENABLE_SHELL
    bool startShell = true;
#else
    bool startShell = false;
#endif
    if (startShell)
    {
        thread_t *shellThd = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, shellThread, &shellCfg);
        chRegSetThreadNameX(shellThd, "shell");
    }
    else
    {
        thread_t *asserCmdSerialThread = chThdCreateStatic(wa_shell, sizeof(wa_shell), LOWPRIO, asservCommandSerial, nullptr);
        chRegSetThreadNameX(asserCmdSerialThread, "asserv Command serial");

        thread_t *controlPanelThd = chThdCreateStatic(wa_controlPanel, sizeof(wa_controlPanel), LOWPRIO, asservPositionSerial, nullptr);
        chRegSetThreadNameX(controlPanelThd, "asserv position update serial");

    }

    deactivateHeapAllocation();

	chThdSetPriority(LOWPRIO);

	while (true)
	{
		//palClearPad(GPIOA, GPIOA_ARD_D8);
		palClearPad(GPIOA, GPIOA_ARD_D12);
		chThdSleepMilliseconds(1000);
		//palSetPad(GPIOA, GPIOA_ARD_D8);
		palSetPad(GPIOA, GPIOA_ARD_D12);
		chThdSleepMilliseconds(1000);

		//debug1("blinking ...\r\n");
	}
}

void asservCommandUSB(BaseSequentialStream *chp, int argc, char **argv)
{
	auto printUsage = []()
	{
		chprintf(outputStream,"Usage :");
		chprintf(outputStream," - asserv en 0|1\r\n");
//		chprintf(outputStream," - asserv enablepolar 0|1\r\n");
			chprintf(outputStream," - asserv coders \r\n");
			chprintf(outputStream," - asserv ext (coders) \r\n");
			chprintf(outputStream," - asserv reset \r\n");
			chprintf(outputStream," - asserv motorspeed [r|l] speed \r\n");
			chprintf(outputStream," -------------- \r\n");
			chprintf(outputStream," - asserv wheelspeedstep [r|l] [speed] [step time] \r\n");
			chprintf(outputStream," -------------- \r\n");
			chprintf(outputStream," - asserv robotfwspeedstep [speed] [step time] \r\n");
//		chprintf(outputStream," - asserv robotangspeedstep [speed] [step time] \r\n");
			chprintf(outputStream," - asserv speedcontrol [r|l] [Kp] [Ki] \r\n");
			chprintf(outputStream," - asserv angleacc delta_speed \r\n");
			chprintf(outputStream," - asserv angleaccdec acc_fw dec_fw acc_bw dec_bw damp \r\n");
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
			chprintf(outputStream," - asserv orbital angleInDeg forward? toTheRight?\r\n");
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
		chprintf(outputStream, "setting wheel %s to speed %.2f rad/s for %d ms \r\n", (side == 'r') ? "right" : "left",
				speedGoal, time);

		float speedRight = speedGoal;
		float speedLeft = 0;
		if (side == 'l')
		{
			speedLeft = speedGoal;
			speedRight = 0;
		}
		bool ok = commandManager->addWheelsSpeed(speedRight, speedLeft, time);
	} else if (!strcmp(argv[0], "robotfwspeedstep"))
	{
		float speedGoal = atof(argv[1]);
		int time = atoi(argv[2]);
		chprintf(outputStream, "setting fw robot speed %.2f rad/s for %d ms\r\n", speedGoal, time);

		bool ok = commandManager->addWheelsSpeed(speedGoal, speedGoal, time);

		/*	} else if (!strcmp(argv[0], "robotangspeedstep"))
		 {
		 float speedGoal = atof(argv[1]);
		 int time = atoi(argv[2]);
		 chprintf(outputStream, "setting angle robot speed %.2f rad/s for %d ms\r\n", speedGoal, time);

		 mainAsserv->setRegulatorsSpeed(0, speedGoal);
		 chThdSleepMilliseconds(time);
		 mainAsserv->setRegulatorsSpeed(0, 0);*/
	} else if (!strcmp(argv[0], "speedcontrol"))
	{
		char side = *argv[1];
		float Kp = atof(argv[2]);
		float Ki = atof(argv[3]);
		uint8_t range = atof(argv[4]);

		chprintf(outputStream, "setting speed control Kp:%.2f Ki:%.2f range:%d to side %c \r\n", Kp, Ki, range, side);

		if (side == 'r')
			speedControllerRight->setGains(Kp, Ki, range);
		else if (side == 'l') speedControllerLeft->setGains(Kp, Ki, range);
	} else if (!strcmp(argv[0], "angleacc"))
	{
		float acc = atof(argv[1]);
		chprintf(outputStream, "setting angle acceleration limit to %.2f \r\n", acc);

		angleAccelerationlimiter->setMaxAcceleration(acc);
	} else if (!strcmp(argv[0], "distacc"))
	{
		float acc_max = atof(argv[1]);
		float acc_min = atof(argv[2]);
		float acc_threshold = atof(argv[3]);
		chprintf(outputStream, "setting distance acceleration limiter max %.2f min %.2f threshold %.2f \r\n", acc_max, acc_min, acc_threshold);
		distanceAccelerationLimiterAdv->setMaxAcceleration(acc_max);
		distanceAccelerationLimiterAdv->setMinAcceleration(acc_min);
		distanceAccelerationLimiterAdv->setHighSpeedThreshold(acc_threshold);
	} else if (!strcmp(argv[0], "distaccdec"))
	{
		float acc_fw = atof(argv[1]);
		float dec_fw = atof(argv[2]);
		float acc_bw = atof(argv[3]);
		float dec_bw = atof(argv[4]);
		float damp = atof(argv[5]);
		chprintf(outputStream,
				"setting distance acceleration dec limiter fw : acc%.2f dec%.2f bw: acc%.2f dec%.2f \r\n", acc_fw,
				dec_fw, acc_bw, dec_bw);
		distanceAccelerationLimiter->setMaxAccFW(acc_fw);
		distanceAccelerationLimiter->setMaxDecFW(dec_fw);
		distanceAccelerationLimiter->setMaxAccBW(acc_bw);
		distanceAccelerationLimiter->setMaxDecBW(dec_bw);
		distanceAccelerationLimiter->setDamplingFactor(damp);
	} else if (!strcmp(argv[0], "addangle"))
	{
		float angle = atof(argv[1]);
		chprintf(outputStream, "Adding angle %.2frad \r\n", angle);
		commandManager->addTurn(angle);
	} else if (!strcmp(argv[0], "anglereset"))
	{
		chprintf(outputStream, "Reseting angle accumulator \r\n");
		angleRegulator->reset();
	} else if (!strcmp(argv[0], "distreset"))
	{
		chprintf(outputStream, "Reseting distance accumulator \r\n");
		distanceRegulator->reset();
	} else if (!strcmp(argv[0], "adddist"))
	{
		float dist = atof(argv[1]);
		bool ok = commandManager->addStraightLine(dist);
		chprintf(outputStream, "Adding distance %.2fmm argc %d (%s)\r\n", dist, argc, argv[1]);

	} else if (!strcmp(argv[0], "anglecontrol"))
	{
		float Kp = atof(argv[1]);
		chprintf(outputStream, "setting angle Kp to %.2f \r\n", Kp);

		angleRegulator->setGain(Kp);
	} else if (!strcmp(argv[0], "distcontrol"))
	{
		float Kp = atof(argv[1]);
		chprintf(outputStream, "setting dist Kp to %.2f \r\n", Kp);

		distanceRegulator->setGain(Kp);
	} else if (!strcmp(argv[0], "wheelsdist"))
	{
		float wheelDistance_mm = atof(argv[1]);
		chprintf(outputStream, "setting wheels dist to %.2f \r\n", wheelDistance_mm);

		mainAsserv->setEncodersWheelsDistance_mm(wheelDistance_mm);
	} else if (!strcmp(argv[0], "en"))
	{
		bool enable = !(atoi(argv[1]) == 0);
		chprintf(outputStream, "%s motor output\r\n", (enable ? "enabling" : "disabling"));
		mainAsserv->enableMotors(enable);
	} else if (!strcmp(argv[0], "coders"))
	{
		float deltaEncoderRight;
		float deltaEncoderLeft;
		encoders->getValues(&deltaEncoderRight, &deltaEncoderLeft);
		chprintf(outputStream, "Encoders count R %d L %d \r\n", encoders->getRightEncoderTotalCount(),
				encoders->getLeftEncoderTotalCount());

	} else if (!strcmp(argv[0], "ext"))
	{
		float deltaEncoderRight;
		float deltaEncoderLeft;
		int32_t encoderRight, encoderLeft;
		encoders_ext->getValues(&deltaEncoderRight, &deltaEncoderLeft);
		encoders_ext->getEncodersTotalCount(&encoderRight, &encoderLeft);
		chprintf(outputStream, "Encoders count R %d  L %d \r\n", encoderRight, encoderLeft);
	} else if (!strcmp(argv[0], "reset"))
	{
		mainAsserv->reset();
		chprintf(outputStream, "asserv resetted \r\n");
	} else if (!strcmp(argv[0], "motorspeed"))
	{
		char side = *argv[1];
		float speedGoal = atof(argv[2]);

		chprintf(outputStream, "setting wheel %s to speed %.2f \r\n", (side == 'r') ? "right" : "left", speedGoal);

		if (side == 'l')
			md22MotorController->setMotorLeftSpeed(speedGoal);
		else
			md22MotorController->setMotorRightSpeed(speedGoal);
//	} else if (!strcmp(argv[0], "enablepolar"))
//	{
//		bool enable = !(atoi(argv[1]) == 0);
//		chprintf(outputStream, "%s polar control\r\n", (enable ? "enabling" : "disabling"));
//
//		mainAsserv->enablePolar(enable);

	} else if (!strcmp(argv[0], "addgoto"))
	{
		float X = atof(argv[1]);
		float Y = atof(argv[2]);
		chprintf(outputStream, "Adding goto(%.2f,%.2f) consign\r\n", X, Y);

		commandManager->addGoTo(X, Y);
	} else if (!strcmp(argv[0], "orbital"))
	{
		float angleInDeg = atof(argv[1]);
		bool forward = !(atoi(argv[2]) == 0);
		float turnToTheRight = atof(argv[3]);

		chprintf(outputStream, "Adding orbital turn of %.2f deg, forward? %d, to the right? %d \r\n", angleInDeg,
				forward, turnToTheRight);

		bool ok = commandManager->addGOrbitalTurn(angleInDeg * M_PI / 180.0, forward, turnToTheRight);
	}

	else if (!strcmp(argv[0], "gototest"))
	{
		mainAsserv->limitMotorControllerConsignToPercentage(50);

		commandManager->addGoTo(200, 0);
		commandManager->addGoTo(300, -50);
		commandManager->addGoTo(400, 100);
		commandManager->addGoTo(400, 300);
		commandManager->addGoTo(0, 300);

		commandManager->addGoTo(0, 0);
		commandManager->addGoToAngle(100, 0);

		commandManager->addGoToNoStop(200, 0);
		commandManager->addGoToNoStop(300, -50);
		commandManager->addGoToNoStop(400, 100);
		commandManager->addGoToNoStop(400, 300);
		commandManager->addGoToNoStop(0, 300);

		commandManager->addGoTo(0, 0);
		commandManager->addGoToAngle(100, 0);

	} else if (!strcmp(argv[0], "get_config"))
	{
		uint8_t index = 0;

		// SpeedControllerLeft
		for (int i = 0; i < NB_PI_SUBSET; i++)
		{
			speedControllerLeft->getGainsForRange(i, &config_buffer[index], &config_buffer[index + 1],
					&config_buffer[index + 2]);
			index += 3;
		}

		// SpeedControllerRight
		for (int i = 0; i < NB_PI_SUBSET; i++)
		{
			speedControllerRight->getGainsForRange(i, &config_buffer[index], &config_buffer[index + 1],
					&config_buffer[index + 2]);
			index += 3;
		}

		//Regulators
		config_buffer[index++] = distanceRegulator->getGain();
		config_buffer[index++] = angleRegulator->getGain();

		// accel limiter
		config_buffer[index++] = angleAccelerationlimiter->getMaxAcceleration();
		config_buffer[index++] = distanceAccelerationLimiterAdv->getMaxAcceleration();
		config_buffer[index++] = distanceAccelerationLimiterAdv->getMinAcceleration();
		config_buffer[index++] = distanceAccelerationLimiterAdv->getHighSpeedThreshold();
		config_buffer[index++] = distanceAccelerationLimiter->getMaxAccFW();
		config_buffer[index++] = distanceAccelerationLimiter->getMaxDecFW();
		config_buffer[index++] = distanceAccelerationLimiter->getMaxAccBW();
		config_buffer[index++] = distanceAccelerationLimiter->getMaxDecBW();
		config_buffer[index++] = distanceAccelerationLimiter->getDamplingFactor();

		chprintf(outputStream, "sending %d float of config !\r\n", index);
		USBStream::instance()->sendConfig((uint8_t*) config_buffer, index * sizeof(config_buffer[0]));
	} else
	{
		printUsage();
	}
}

//THD_FUNCTION(ControlPanelThread, p)
//{
//	(void) p;
//	void *ptr = nullptr;
//	uint32_t size = 0;
//	char *firstArg = nullptr;
//	char *argv[7];
//	while (!chThdShouldTerminateX())
//	{
//		USBStream::instance()->getFullBuffer(&ptr, &size);
//		if (size > 0)
//		{
//			char *buffer = (char*) ptr;
//			buffer[size] = 0;
//			/*
//			 *  On transforme la commande recu dans une version argv/argc
//			 *    de manière a utiliser les commandes shell déjà définie...
//			 */
//			bool prevWasSpace = false;
//			firstArg = buffer;
//			int nb_arg = 0;
//			for (uint32_t i = 0; i < size; i++)
//			{
//				if (prevWasSpace && buffer[i] != ' ')
//				{
//					argv[nb_arg++] = &buffer[i];
//				}
//
//				if (buffer[i] == ' ' || buffer[i] == '\r' || buffer[i] == '\n')
//				{
//					prevWasSpace = true;
//					buffer[i] = '\0';
//				} else
//				{
//					prevWasSpace = false;
//				}
//			}
//
//			// On évite de faire appel au shell si le nombre d'arg est mauvais ou si la 1ière commande est mauvaise...
//			if (nb_arg > 0 && !strcmp(firstArg, "asserv"))
//			{
//				asservCommandUSB(nullptr, nb_arg, argv);
//			}
//			USBStream::instance()->releaseBuffer();
//		}
//	}
//}

void usbSerialCallback(char *buffer, uint32_t size)
{
	if (size > 0)
	{
		/*
		 *  On transforme la commande recu dans une version argv/argc
		 *    de manière a utiliser les commandes shell déjà définie...
		 */
		bool prevWasSpace = false;
		char *firstArg = buffer;
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
				//buffer[i] = '\0';
				buffer[i] = 0;
			} else
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
