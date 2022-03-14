#include "AccelerationDecelerationLimiter.h"
#include <algorithm>
#include <math.h>
#include "USBStream.h"


AccelerationDecelerationLimiter::AccelerationDecelerationLimiter(float maxAcceleration, float maxSpeed, float positionCorrectorKp, bool isAngleLimiter) : AccelerationLimiter()
{
	m_enabled = true;
	m_initialPositionError = 0;
	m_previousPositionGoal = 0;
	m_previousLimitedOutput = 0;
	m_velocityCompensation = 0;
	m_CompensatedOutput = 0;
	m_velocityAtDecTime = 0;
    m_maxAcceleration = maxAcceleration;
    m_maxSpeed = maxSpeed;
    m_positionCorrectorKp = positionCorrectorKp;
    m_isAngleLimiter = isAngleLimiter;
}

float AccelerationDecelerationLimiter::limitAcceleration(float dt, float targetSpeed, float , float positionGoal, float positionError)
{
	if (!m_enabled)
		return targetSpeed;

	if( m_previousPositionGoal != positionGoal )
	{
		m_initialPositionError = positionError;
		m_previousPositionGoal = positionGoal;
	}

	// 1st compute the velocity estimator at deceleration time
	m_velocityAtDecTime = std::min( sqrt(m_maxAcceleration*m_initialPositionError*1.1f) , m_maxSpeed );
	if( m_velocityAtDecTime == 0)
		m_velocityAtDecTime = 1e-5; // In this case, avoid division by zero !

	m_velocityCompensation = ((m_positionCorrectorKp*1.1f)/(2.0*m_maxAcceleration) - 1.0/m_velocityAtDecTime ) * (m_previousLimitedOutput*m_previousLimitedOutput);

	m_CompensatedOutput = targetSpeed - m_velocityCompensation;

	float max_delta = m_maxAcceleration*dt;
	if(m_CompensatedOutput > m_previousLimitedOutput+max_delta)
		m_CompensatedOutput = m_previousLimitedOutput+max_delta;
	if(m_CompensatedOutput < m_previousLimitedOutput-max_delta)
		m_CompensatedOutput = m_previousLimitedOutput-max_delta;

	if( m_isAngleLimiter )
	{
//		USBStream::instance()->setAngleLimiterVelocityAtDecTime(m_velocityAtDecTime);
//		USBStream::instance()->setAngleLimiterVelocityCompensation(m_velocityCompensation);
//		USBStream::instance()->setAngleLimiterVelocityCompensated(targetSpeed - m_velocityCompensation);
//		USBStream::instance()->setAngleLimiterOutput(m_CompensatedOutput);
	}
	else
	{
		USBStream::instance()->setDistanceLimiterVelocityAtDecTime(m_velocityAtDecTime);
		USBStream::instance()->setDistanceLimiterVelocityCompensation(m_velocityCompensation);
		USBStream::instance()->setDistanceLimiterVelocityCompensated(targetSpeed - m_velocityCompensation);
		USBStream::instance()->setDistanceLimiterOutput(m_CompensatedOutput);
		USBStream::instance()->setDistanceLimiterInitialPosError(m_initialPositionError);
		USBStream::instance()->setDistanceLimiterTargetSpeed(targetSpeed);
	}

	m_previousLimitedOutput = m_CompensatedOutput;
	return m_CompensatedOutput;
}

void AccelerationDecelerationLimiter::enable()
{
	   m_enabled = true;
}

void AccelerationDecelerationLimiter::disable()
{
	m_enabled = false;
}

void AccelerationDecelerationLimiter::reset()
{
	m_initialPositionError = 0;
	m_previousPositionGoal = 0;
	m_previousLimitedOutput = 0;
	m_velocityAtDecTime = 0;
	m_velocityCompensation = 0;
	m_CompensatedOutput = 0;
}

