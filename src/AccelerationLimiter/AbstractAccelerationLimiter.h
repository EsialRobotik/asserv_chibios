#ifndef SRC_ACCELERATIONLIMITER_ABSTRACTACCELERATIONLIMITER_H_
#define SRC_ACCELERATIONLIMITER_ABSTRACTACCELERATIONLIMITER_H_

#include "AccelerationLimiter.h"

class AbstractAccelerationLimiter : public AccelerationLimiter
{

public:
    AbstractAccelerationLimiter();
    virtual ~AbstractAccelerationLimiter(){};

    virtual float limitAcceleration(float dt, float targetSpeed, float currentSpeed);

    virtual void enable();
    virtual void disable();

    virtual void reset();

    static float constrain(float value, float low, float high);

private:

    virtual float limitOutput(float dt, float targetSpeed, float previousOutput, float currentSpeed) = 0;

    bool  m_enabled;
    float m_lastOutput;
};

#endif /* SRC_ACCELERATIONLIMITER_ABSTRACTACCELERATIONLIMITER_H_ */
