#include "AccelerationDecelerationLimiter.h"
#include <algorithm>
#include <math.h>
#include "USBStream.h"


AccelerationDecelerationLimiter::AccelerationDecelerationLimiter(
		float maxAcceleration, float maxDeceleration,
		float maxSpeed, float positionCorrectorKp, bool isAngleLimiter)
	: AccelerationDecelerationLimiter(maxAcceleration, maxDeceleration, maxAcceleration, maxDeceleration, maxSpeed, positionCorrectorKp, isAngleLimiter)
{
}

AccelerationDecelerationLimiter::AccelerationDecelerationLimiter(
		float maxAccelerationForward, float maxDecelerationForward,
		float maxAccelerationBackward, float maxDecelerationBackward,
		float maxSpeed, float positionCorrectorKp, bool isAngleLimiter): AccelerationLimiter()
{
	m_enabled = true;
	m_initialPositionError = 0;
	m_previousPositionGoal = 0;
	m_previousLimitedOutput = 0;
	m_velocityCompensation = 0;
	m_CompensatedOutput = 0;
	m_velocityAtDecTime = maxSpeed;
    m_maxAccelerationForward = maxAccelerationForward;
    m_maxDecelerationForward = maxDecelerationForward;
    m_maxAccelerationBackward = maxAccelerationBackward;
    m_maxDecelerationBackward = maxDecelerationBackward;
    m_maxSpeed = maxSpeed;
    m_positionCorrectorKp = positionCorrectorKp;
    m_isAngleLimiter = isAngleLimiter;
    m_damplingFactor = 2.8f;
}

float AccelerationDecelerationLimiter::limitAcceleration(float dt, float targetSpeed, float currentSpeed, float positionGoal, float positionError)
{
	if (!m_enabled)
		return targetSpeed;

	float maxAcceleration = m_maxAccelerationForward;
	float maxDeceleration = m_maxDecelerationForward;
	float way = 1.0f;
	bool forward = true;
	if(positionError < 0.0f)
	{
		maxAcceleration = m_maxAccelerationBackward;
		maxDeceleration = m_maxDecelerationBackward;
		way = -1.0f;
		forward = false;
	}

	if( fabs(m_previousPositionGoal-positionGoal) > 5   )
	{
		m_initialPositionError = positionError;
		m_previousPositionGoal = positionGoal;

		// compute the velocity estimator at deceleration time
		if (m_initialPositionError != 0)
			m_velocityAtDecTime = std::min( sqrt(fabs(maxAcceleration*m_initialPositionError*m_damplingFactor)) , m_maxSpeed );
		else
			m_velocityAtDecTime = m_maxSpeed;
	}

	m_velocityCompensation = way*((m_positionCorrectorKp*m_damplingFactor)/(2.0*maxDeceleration) - 1.0/m_velocityAtDecTime ) * (m_previousLimitedOutput*m_previousLimitedOutput);

	m_CompensatedOutput = targetSpeed - m_velocityCompensation;

	float max_delta_up = (forward) ? (maxAcceleration*dt) : (maxDeceleration*dt);
	float max_delta_down = (forward) ? (maxDeceleration*dt) : (maxAcceleration*dt);

	if(m_CompensatedOutput > m_previousLimitedOutput+max_delta_up)
		m_CompensatedOutput = m_previousLimitedOutput+max_delta_up;
	if(m_CompensatedOutput < m_previousLimitedOutput-max_delta_down)
		m_CompensatedOutput = m_previousLimitedOutput-max_delta_down;

	if( m_CompensatedOutput > m_maxSpeed)
		m_CompensatedOutput = m_maxSpeed;
	else if( m_CompensatedOutput < -m_maxSpeed)
		m_CompensatedOutput = -m_maxSpeed;

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
		USBStream::instance()->setDistanceLimitercurrentSpeed(currentSpeed);
		USBStream::instance()->setDistanceLimiterInitialPosError(m_initialPositionError);
		USBStream::instance()->setDistanceLimiterTargetSpeed(targetSpeed);
		USBStream::instance()->setDistanceLimiterMaxAcc(maxAcceleration);
		USBStream::instance()->setDistanceLimiterMaxDec(maxDeceleration);
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

