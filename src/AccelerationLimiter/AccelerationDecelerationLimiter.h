#ifndef SRC_ACCELERATIONDECELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_
#define SRC_ACCELERATIONDECELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_

#include "AccelerationLimiter.h"

class AccelerationDecelerationLimiter : public AccelerationLimiter
{

public:
    explicit AccelerationDecelerationLimiter(float maxAcceleration, float maxDeceleration, float maxSpeed, float positionCorrectorKp, bool isAngleLimiter);
    virtual ~AccelerationDecelerationLimiter(){};

    virtual float limitAcceleration(float dt, float targetSpeed, float currentSpeed, float positionGoal, float positionError);

    virtual void enable();
    virtual void disable();
    virtual void reset();

private:

    bool  m_enabled;
    float m_initialPositionError;
    float m_previousPositionGoal;
    float m_maxAccelerationForward;
    float m_maxDecelerationForward;
    float m_maxAccelerationBackward;
    float m_maxDecelerationBackward;
    float m_maxSpeed;
    float m_positionCorrectorKp;
    float m_previousLimitedOutput;
    float m_velocityCompensation;
    float m_CompensatedOutput;
    float m_velocityAtDecTime;
    bool m_isAngleLimiter;
};

#endif /* SRC_ACCELERATIONDECELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_ */
