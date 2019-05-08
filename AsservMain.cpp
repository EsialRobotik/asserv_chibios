/*
 * AsservMain.cpp
 *
 *  Created on: 4 mai 2019
 *      Author: jeff
 */

#include "AsservMain.h"
#include "ch.h"
#include "hal.h"

#define M_PI 3.14159265358979323846

#define ASSERV_THREAD_PERIOD_MS (1)
#define ASSERV_THREAD_PERIOD_S (float(ASSERV_THREAD_PERIOD_MS)/1000.0)


AsservMain::AsservMain():
m_motorController(true,false),
m_encoders(true,true),
m_speedControllerRight(15, 5, 100, 23.0, 100, 1.0/ASSERV_THREAD_PERIOD_S),
m_speedControllerLeft(15, 5, 100, 23.0, 100, 1.0/ASSERV_THREAD_PERIOD_S)
{
}

AsservMain::~AsservMain()
{
}


void AsservMain::init()
{
	m_motorController.init();
	m_encoders.init();
	m_encoders.start();
	USBStream::init();
}


float AsservMain::estimateSpeed(int16_t deltaCount)
{
	const float ticksByTurn = 1024;
	const float dt = ASSERV_THREAD_PERIOD_S;

	float deltaAngleRadian = ((float)deltaCount/(float)ticksByTurn);
	float speedRadian = deltaAngleRadian / dt;
	return speedRadian;
}


void AsservMain::mainLoop()
{
	systime_t time = chVTGetSystemTime();
	time += TIME_MS2I(ASSERV_THREAD_PERIOD_MS);
	while (true)
	{
		int16_t m_encoderDeltaRight;
		int16_t m_encoderDeltaLeft;
		m_encoders.getValuesAndReset(&m_encoderDeltaLeft, &m_encoderDeltaRight);

		float estimatedSpeedRight = estimateSpeed(m_encoderDeltaRight);
		float estimatedSpeedLeft = estimateSpeed(m_encoderDeltaLeft);

		float outputSpeedRight = m_speedControllerRight.update(estimatedSpeedRight);
		float outputSpeedLeft = m_speedControllerLeft.update(estimatedSpeedLeft);

		m_motorController.setMotor2Speed(outputSpeedRight);
		m_motorController.setMotor1Speed(outputSpeedLeft);

		USBStream::instance()->setSpeedEstimatedRight(estimatedSpeedRight);
		USBStream::instance()->setSpeedEstimatedLeft(estimatedSpeedLeft);
		USBStream::instance()->setSpeedGoalRight(m_speedControllerRight.getSpeedGoal());
		USBStream::instance()->setSpeedGoalLeft(m_speedControllerLeft.getSpeedGoal());
		USBStream::instance()->setSpeedOutputRight(outputSpeedRight);
		USBStream::instance()->setSpeedOutputLeft(outputSpeedLeft);
		USBStream::instance()->setSpeedIntegratedOutputRight(m_speedControllerRight.getIntegratedOutput());
		USBStream::instance()->setSpeedIntegratedOutputLeft(m_speedControllerLeft.getIntegratedOutput());
		USBStream::instance()->setLimitedSpeedGoalRight(m_speedControllerRight.getLimitedSpeedGoal());
		USBStream::instance()->setLimitedSpeedGoalLeft(m_speedControllerLeft.getLimitedSpeedGoal());
		USBStream::instance()->SendCurrentStream();

		chThdSleepUntil(time);
		time += TIME_MS2I(ASSERV_THREAD_PERIOD_MS);
	}
}

void AsservMain::setMotorsSpeed(float motorLeft, float motorRight)
{
	m_speedControllerRight.setSpeedGoal(motorRight);
	m_speedControllerLeft.setSpeedGoal(motorLeft);
}

