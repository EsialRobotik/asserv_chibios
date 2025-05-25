#include "SimpleAccelerationLimiter.h"

SimpleAccelerationLimiter::SimpleAccelerationLimiter(float maxAcceleration) : AbstractAccelerationLimiter()
{
    m_maxAcceleration = maxAcceleration;
}

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


Cbore & SimpleAccelerationLimiter::getConfiguration(Cbore & cbor_representation)
{
    return cbor_representation.map()
            .key("name").value("acc_limiter")
            .key("max_acc").value(m_maxAcceleration)
            .end();
}
