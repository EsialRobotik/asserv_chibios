#include "SpeedController.h"
#include <cmath>
#include <cstdlib>


SpeedController::SpeedController(float speedKp, float speedKi, float outputLimit, float maxInputSpeed, float maxDeltaConsign, float measureFrequency)
{
	m_speedGoal = 0;
	m_limitedSpeedGoal = 0;
	m_limitedSpeedGoalPrev = 0;
	m_integratedOutput = 0;
	m_speedKp = speedKp;
	m_speedKi = speedKi;
	m_outputLimit = outputLimit;
	m_inputLimit = maxInputSpeed;
	m_deltaConsignMax = maxDeltaConsign;
	m_measureFrequency = measureFrequency;
}

float SpeedController::update(float actualSpeed)
{
	float outputValue = 0;
	m_limitedSpeedGoal = m_speedGoal;

	if( m_speedGoal > 0 && m_limitedSpeedGoal > ((m_limitedSpeedGoalPrev+m_deltaConsignMax)) )
		m_limitedSpeedGoal = m_limitedSpeedGoalPrev+m_deltaConsignMax;
	else if (m_speedGoal < 0 && m_limitedSpeedGoal < ((m_limitedSpeedGoalPrev-m_deltaConsignMax)) )
		m_limitedSpeedGoal = m_limitedSpeedGoalPrev-m_deltaConsignMax;

	m_limitedSpeedGoalPrev = m_limitedSpeedGoal;

	float speedError = m_limitedSpeedGoal - actualSpeed;

	// Regulateur en vitesse : un PI
	outputValue = speedError*m_speedKp;
	outputValue += m_integratedOutput;


	// On limite la sortie entre -m_outputLimit et m_outputLimit...
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


	if(limited) // .. Si la sortie est limité, on désature l'intégrale
	{
		m_integratedOutput *= 0.9;
	}
	else	// .. Sinon, on integre l'erreur
	{
		m_integratedOutput +=  m_speedKi * speedError/m_measureFrequency;
		if( std::fabs(speedError) < 0.1) // Quand l'erreur de vitesse est proche de zero(ie: consigne à 0 et le robot ne bouge pas..), on désature l'intégrale
			m_integratedOutput *= 0.95;
	}

	// Protection antiWindup, surement inutile avec la désaturation au dessus, mais on garde ceinture & bretelles !
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

	if(speed == 0.0)
		resetIntegral();
	m_speedGoal = speed;
};

void SpeedController::setGains(float Kp, float Ki)
{
	m_speedKp = Kp;
	m_speedKi = Ki;
	resetIntegral();
}
