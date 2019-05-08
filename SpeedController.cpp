/*
 * SpeedController.cpp
 *
 *  Created on: 29 avr. 2019
 *      Author: jeff
 */

#include "SpeedController.h"

SpeedController::SpeedController(float speedKp, float speedKi, float outputLimit, float maxInputSpeed, float measureFrequency)
{
	m_speedGoal = 0;
	m_limitedSpeedGoal = 0;
	m_limitedSpeedGoalPrev = 0;
	m_integratedOutput = 0;
	m_speedKp = speedKp;
	m_speedKi = speedKi;
	m_outputLimit = outputLimit;
	m_inputLimit = maxInputSpeed;
	m_deltaConsignMax = 0.1;
	m_measureFrequency = measureFrequency;
}

float SpeedController::update(float actualSpeed)
{
	float outputValue = 0;
	m_limitedSpeedGoal = m_speedGoal;
	if( m_speedGoal > (m_limitedSpeedGoalPrev+m_deltaConsignMax))
		m_limitedSpeedGoal = m_limitedSpeedGoalPrev+m_deltaConsignMax;
	else if( m_speedGoal < (m_limitedSpeedGoalPrev-m_deltaConsignMax))
		m_limitedSpeedGoal = m_limitedSpeedGoalPrev-m_deltaConsignMax;
	m_limitedSpeedGoalPrev = m_limitedSpeedGoal;

	float speedError = m_limitedSpeedGoal - actualSpeed;

	// Speed controller is PI only
	outputValue = speedError*m_speedKp;
	outputValue += m_integratedOutput;


	// Bound output value to correct value
	bool limited = false;
	if (outputValue > m_outputLimit)
	{
		outputValue = m_outputLimit;
		limited = true;
	}
	if (outputValue < -m_outputLimit)
	{
		outputValue = -m_outputLimit;
		limited = true;
	}

	if(limited)
	{
		m_integratedOutput *= 0.9;
	}
	else
	{
		m_integratedOutput +=  m_speedKi * speedError/m_measureFrequency;
	}
	// Anti Windup protection, probably useless with the saturation handling below..
	if(m_integratedOutput > m_outputLimit)
		m_integratedOutput = m_outputLimit;
	else if(m_integratedOutput < -m_outputLimit)
		m_integratedOutput = -m_outputLimit;

	return outputValue;
}

void SpeedController::setSpeedGoal(float speed)
{
	if(speed > m_inputLimit)
		speed = m_inputLimit;
	if(speed < -m_inputLimit)
		speed = -m_inputLimit;

	m_speedGoal = speed;
};

void SpeedController::setGains(float Kp, float Ki)
{
	m_speedKp = Kp;
	m_speedKi = Ki;
	resetIntegral();
}
