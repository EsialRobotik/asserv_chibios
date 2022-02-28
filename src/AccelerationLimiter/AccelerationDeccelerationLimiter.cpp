#include "AccelerationDeccelerationLimiter.h"
#include <cmath>
#include "ch.h"
#include "USBStream.h"


AccelerationDeccelerationLimiter::AccelerationDeccelerationLimiter(float maxAcceleration, float maxDecceleration, float arrivalWindow, float maxSpeed, const Regulator &regulator)
: m_regulator(regulator), m_enabled(true)
{
    m_maxAcceleration = maxAcceleration;
    m_maxDecceleration = maxDecceleration;
    m_arrivalWindow = arrivalWindow;
    m_maxSpeed = maxSpeed;
    m_currentSpeedConsign = 0;
    m_prevOutputPositionConsign = 0;
    chDbgAssert(maxDecceleration < 0, "maxDecceleration must be negative");
    chDbgAssert(maxAcceleration > 0, "maxAcceleration must be negative");
}

float AccelerationDeccelerationLimiter::limitAccelerationDecceleration(float positionConsign, float currentPosition, float currentSpeed, float dt)
{
    float stoppingDistance = (currentSpeed * currentSpeed) / (2.0 * -m_maxDecceleration);
    float positionError = positionConsign - currentPosition;
    float positionErrorAbs = fabs(positionError);

    float way = 1; // forward
    if(positionError < 0)
        way = -1; // backward

    float outputPositionConsign;

    USBStream::instance()->setSlope(0);
    if( positionErrorAbs < m_arrivalWindow) // In this we are to close to the goal, just let the regulator do its job
    {
        m_currentSpeedConsign = way*positionError;
    }
    else if( positionErrorAbs > stoppingDistance) // In this case accelerate or travel at constant speed
    {
        float maxSpeedConsign = m_maxSpeed / m_regulator.getGain();
        m_currentSpeedConsign += m_maxAcceleration / m_regulator.getGain() * dt;

        if( m_currentSpeedConsign > maxSpeedConsign)
            m_currentSpeedConsign = maxSpeedConsign;
    }
    else // In this case, try to make a smooth deceleration !
    {
//        float stoppingTime = currentSpeed / m_maxDecceleration;
//        float slope = (positionConsign-currentPosition)/stoppingTime;
//
//        USBStream::instance()->setSlope(slope);
//        USBStream::instance()->setStoppingTime(stoppingTime);
////        m_currentSpeedConsign += slope / m_regulator.getGain() * dt;

        m_currentSpeedConsign += m_maxDecceleration/m_regulator.getGain() * dt;
    }

    if( m_currentSpeedConsign > positionErrorAbs)
        m_currentSpeedConsign = positionErrorAbs;

    outputPositionConsign =  currentPosition + way*m_currentSpeedConsign;

    USBStream::instance()->setSpeedConsignForAccLimit(m_currentSpeedConsign);
    USBStream::instance()->setstoppingDistance(stoppingDistance);
    USBStream::instance()->setDistanceSpeed(currentSpeed);

    m_prevOutputPositionConsign = outputPositionConsign;
    return outputPositionConsign;
}

