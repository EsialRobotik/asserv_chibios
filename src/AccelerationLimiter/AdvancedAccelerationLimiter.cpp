#include "AdvancedAccelerationLimiter.h"
#include <cmath>


AdvancedAccelerationLimiter::AdvancedAccelerationLimiter(float maxAcceleration, float minAcceleration, float highSpeedThreshold) :
AbstractAccelerationLimiter()
{
    m_maxAcceleration = maxAcceleration;
    m_minAcceleration = minAcceleration;
    m_HighSpeedThreshold = highSpeedThreshold;
}

/*
 * Advanced acceleration limiter is a bit more complex!
 *
 *   When the robot is accelerating
 *   ( determined by the upper part of the class, ie: AbstractAccelerationLimiter),
 *    the next output speed consign is limited considering the current speed.
 *
 *  As the robot can accelerate more when it's already moving than when it's almost stationary.
 *  So we determine a max acceleration that will be used when the robot speed reach highSpeedThreshold.
 *  Below this speed, a linear interpolation between minAcceleration and maxAcceleration will be use !
 *
 *  This implementation will limit the jerk of the robot.
 */

float AdvancedAccelerationLimiter::limitOutput(float dt, float targetSpeed, float previousOutput, float currentSpeed)
{
    float change = targetSpeed - previousOutput;
    float maxDelta;

    if( fabs(currentSpeed) >= m_HighSpeedThreshold)
       maxDelta = dt * m_maxAcceleration;
   else
       maxDelta = dt * (m_minAcceleration + fabs(currentSpeed)*(m_maxAcceleration-m_minAcceleration)/m_HighSpeedThreshold);

    return constrain(change, -maxDelta, maxDelta);
}

void AdvancedAccelerationLimiter::setMaxAcceleration(float maxAcceleration)
{
    m_maxAcceleration = maxAcceleration;
}

void AdvancedAccelerationLimiter::setMinAcceleration(float minAcceleration)
{
    m_minAcceleration = minAcceleration;
}

void AdvancedAccelerationLimiter::setHighSpeedThreshold(float highSpeedThreshold)
{
    m_HighSpeedThreshold = highSpeedThreshold;
}
