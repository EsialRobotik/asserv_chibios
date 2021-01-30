#include "SimpleAccelerationLimiter.h"

SimpleAccelerationLimiter::SimpleAccelerationLimiter(float maxAcceleration) : AbstractAccelerationLimiter()
{
    m_maxAcceleration = maxAcceleration;
}

/*
 * Simple acceleration limiter is ... Simple !
 *
 *   When the robot is accelerating
 *   ( determined by the upper part of the class, ie: AbstractAccelerationLimiter),
 *    the next output speed consign is limited by m_maxAcceleration.
 *
 *  Output speed will be a trapezoidal shape velocity
 */

float SimpleAccelerationLimiter::limitOutput(float dt, float targetSpeed, float previousOutput, float)
{
    float change = targetSpeed - previousOutput;
    float maxDelta = dt * m_maxAcceleration;

    return constrain(change, -maxDelta, maxDelta);
}


void SimpleAccelerationLimiter::setMaxAcceleration(float maxAcceleration)
{
    m_maxAcceleration = maxAcceleration;
}
