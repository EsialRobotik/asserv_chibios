#ifndef SRC_ACCELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_
#define SRC_ACCELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_

#include "AbstractAccelerationLimiter.h"

class SimpleAccelerationLimiter : public AbstractAccelerationLimiter
{

public:
    explicit SimpleAccelerationLimiter(float maxAcceleration);
    virtual ~SimpleAccelerationLimiter(){};

    void setMaxAcceleration(float maxAcceleration);
    inline float getMaxAcceleration() const { return m_maxAcceleration; };

private:
    virtual float limitOutput(float dt, float targetSpeed, float previousOutput, float currentSpeed);

    float m_maxAcceleration;
};

#endif /* SRC_ACCELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_ */
