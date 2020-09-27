#include "SlopeFilter.h"
#include <cfloat>
#include <cmath>

SlopeFilter::SlopeFilter(float maxAcceleration, float maxAccelerationLowSpeed, float lowSpeecThreshold)
{
    m_maxAcceleration = maxAcceleration;
    m_maxDeceleration = FLT_MAX;
    m_maxAccelerationLowSpeed = maxAccelerationLowSpeed;
    m_lowSpeedThreshold = lowSpeecThreshold;
    m_lastOutput = 0;
    m_enabled = true;
}

float SlopeFilter::filter(float dt, float targetSpeed, float currentSpeed)
{
    if( !m_enabled)
        return targetSpeed;

    bool isAccelerating;
    if( m_lastOutput >= 0 && targetSpeed >= 0)
    {
        if (m_lastOutput <= targetSpeed)
            isAccelerating = true;
        else
            isAccelerating = false;
    }
    else if( m_lastOutput < 0 && targetSpeed < 0)
    {
        if (m_lastOutput <= targetSpeed)
            isAccelerating = false;
        else
            isAccelerating = true;
    }
    else // In this case we will change way (ie: going front to going back), consider this as acceleration
    {
        isAccelerating = true;
    }


    float change = targetSpeed - m_lastOutput;
    float maxDelta;
    if( isAccelerating )
    {
        if( fabs(currentSpeed) >= m_lowSpeedThreshold)
            maxDelta = dt * m_maxAcceleration;
        else
            maxDelta = dt * (m_maxAccelerationLowSpeed + fabs(currentSpeed)*(m_maxAcceleration-m_maxAccelerationLowSpeed)/m_lowSpeedThreshold);
    }
    else
    {
        maxDelta = dt * m_maxDeceleration;
    }

    m_lastOutput += constrain(change, -maxDelta, maxDelta);
    return m_lastOutput;
}


void SlopeFilter::setSlope(float slope)
{
    m_maxAcceleration = slope;
    m_lastOutput = 0;
}

void SlopeFilter::enable()
{
    m_enabled = true;
}

void SlopeFilter::disable()
{
    m_enabled = false;
}

float SlopeFilter::constrain(float value, float low, float high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}
