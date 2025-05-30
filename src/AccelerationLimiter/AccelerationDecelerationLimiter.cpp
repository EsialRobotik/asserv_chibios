#include "AccelerationDecelerationLimiter.h"
#include <algorithm>
#include <math.h>
#include "sampleStream/SampleStreamInterface.h"



AccelerationDecelerationLimiter::AccelerationDecelerationLimiter(
        float maxAcceleration, float maxDeceleration,
        float maxSpeed, float dampling, float positionCorrectorKp)
    : AccelerationDecelerationLimiter(maxAcceleration, maxDeceleration, maxAcceleration, maxDeceleration, maxSpeed, dampling, positionCorrectorKp)
{
}

AccelerationDecelerationLimiter::AccelerationDecelerationLimiter(
        float maxAccelerationForward, float maxDecelerationForward,
        float maxAccelerationBackward, float maxDecelerationBackward,
        float maxSpeed, float dampling, float positionCorrectorKp): AccelerationDecelerationLimiterInterface()
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
    m_maxAttainableSpeed = 0;
    m_damplingFactor = dampling;
    m_timeToVmax = 0;
    m_timeFromVmaxToZero = 0;
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


    if( m_previousPositionGoal != positionGoal   )
    {
        // An new position goal in inputed, try to compute the velocity at deceleration time,
        //  using the init speed, the maximum acceleration/deceleration and the error at startup
        //  If the target is far enough to go to the maximum speed, just use the max speed

        m_initialPositionError = positionError;
        m_previousPositionGoal = positionGoal;

        m_timeToVmax = (m_maxSpeed-fabs(currentSpeed))/maxAcceleration;
        m_timeFromVmaxToZero = m_maxSpeed/maxDeceleration;
        m_maxAttainableSpeed = sqrt( (2.0f*maxDeceleration * (fabs(positionError)*2.0f*maxAcceleration + currentSpeed*currentSpeed) ) / (2.0f*maxDeceleration + 2.0f*maxAcceleration)  );

        m_velocityAtDecTime = std::min( m_maxAttainableSpeed , m_maxSpeed );
    }

    m_velocityCompensation = way*((m_positionCorrectorKp*m_damplingFactor)/(2.0*maxDeceleration) - 1.0/m_velocityAtDecTime ) * (m_previousLimitedOutput*m_previousLimitedOutput);

    m_CompensatedOutput = targetSpeed - m_velocityCompensation;


    // Apply a basic "slope filter" to limit maximum acceleration and deceleration
    float max_delta_up = (forward) ? (maxAcceleration*dt) : (maxDeceleration*dt);
    float max_delta_down = (forward) ? (maxDeceleration*dt) : (maxAcceleration*dt);

    if(m_CompensatedOutput > m_previousLimitedOutput+max_delta_up)
        m_CompensatedOutput = m_previousLimitedOutput+max_delta_up;
    
    if(m_CompensatedOutput < m_previousLimitedOutput-max_delta_down)
        m_CompensatedOutput = m_previousLimitedOutput-max_delta_down;

    // Also limit the maximum speed consign
    if( m_CompensatedOutput > m_maxSpeed)
        m_CompensatedOutput = m_maxSpeed;
    if( m_CompensatedOutput < -m_maxSpeed)
        m_CompensatedOutput = -m_maxSpeed;

   if( targetSpeed > 0 && m_CompensatedOutput > targetSpeed)
       m_CompensatedOutput = targetSpeed;

   if( targetSpeed < 0 && m_CompensatedOutput < targetSpeed)
       m_CompensatedOutput = targetSpeed;



    SampleStream *instance = SampleStream::instance();
    instance->setDistanceLimiterVelocityAtDecTime(m_velocityAtDecTime);
    instance->setDistanceLimiterVelocityCompensation(m_velocityCompensation);
    instance->setDistanceLimiterVelocityCompensated(targetSpeed - m_velocityCompensation);
    instance->setDistanceLimiterOutput(m_CompensatedOutput);
    instance->setDistanceLimitercurrentSpeed(currentSpeed);
    instance->setDistanceLimiterTargetSpeed(targetSpeed);
    instance->setDistanceLimiterTimeToVMax(m_timeToVmax);
    instance->setDistanceLimiterMaxAttainableSpeed(m_maxAttainableSpeed);
    instance->setDistanceLimiterTimeFromVmaxToZero(m_timeFromVmaxToZero);
    instance->setMaxAcceleration(maxAcceleration);
    instance->setMaxDeceleration(maxDeceleration);
    instance->setMaxDeltaUp(max_delta_up);
    instance->setMaxDeltaDown(max_delta_down);


    m_previousLimitedOutput = m_CompensatedOutput;
    return m_CompensatedOutput;
}


void AccelerationDecelerationLimiter::enable()
{
    if( !m_enabled)
        reset();  // When enabling again this limiter, reset the internal values. This should fix the famous bug "the moustache of rami"

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
    m_velocityAtDecTime = m_maxSpeed;
    m_velocityCompensation = 0;
    m_CompensatedOutput = 0;
    m_maxAttainableSpeed = 0;
}

