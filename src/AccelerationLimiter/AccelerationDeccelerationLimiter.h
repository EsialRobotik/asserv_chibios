#ifndef SRC_ACCELERATIONDECCELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_
#define SRC_ACCELERATIONDECCELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_

#include "Regulator.h"

class AccelerationDeccelerationLimiter
{

public:
    explicit AccelerationDeccelerationLimiter(float maxAcceleration, float maxDecceleration, float arrivalWindow, float maxSpeed, const Regulator &regulator);
    virtual ~AccelerationDeccelerationLimiter(){};

    float limitAccelerationDecceleration(float positionConsign, float currentPosition, float currentSpeed, float dt);

    inline float getLastOutputConsign() const { return m_prevOutputPositionConsign; };

private:
    const Regulator &m_regulator;
    float m_maxAcceleration;
    float m_maxDecceleration;
    float m_arrivalWindow;
    float m_maxSpeed;

    float m_currentSpeedConsign;
    float m_prevOutputPositionConsign;

};

#endif /* SRC_ACCELERATIONDECCELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_ */
