#ifndef SRC_ACCELERATIONDECELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_
#define SRC_ACCELERATIONDECELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_

#include "AccelerationDecelerationLimiterInterface.h"
#include "sampleStream/configuration/ConfigurationInterface.h"


class AccelerationDecelerationLimiter : public AccelerationDecelerationLimiterInterface, public Configuration
{

public:
    explicit AccelerationDecelerationLimiter(float maxAcceleration, float maxDeceleration, float maxSpeed, float dampling, float positionCorrectorKp);
    explicit AccelerationDecelerationLimiter(float maxAccelerationForward, float maxDecelerationForward, float maxAccelerationBackward, float maxDecelerationBackward, float maxSpeed, float dampling, float positionCorrectorKp);
    virtual ~AccelerationDecelerationLimiter(){};

    virtual float limitAcceleration(float dt, float targetSpeed, float currentSpeed, float positionGoal, float positionError);

    virtual void enable();
    virtual void disable();
    virtual void reset();

    inline void setDamplingFactor(float value ){m_damplingFactor = value;};
    inline float getDamplingFactor() const {return m_damplingFactor;};


    inline float getMaxAccFW() const { return m_maxAccelerationForward; };
    inline float getMaxDecFW() const { return m_maxDecelerationForward; };
    inline float getMaxAccBW() const { return m_maxAccelerationBackward; };
    inline float getMaxDecBW() const { return m_maxDecelerationBackward; };

    inline void setMaxAccFW(float value)  { m_maxAccelerationForward = value; };
    inline void setMaxDecFW(float value)  { m_maxDecelerationForward = value; };
    inline void setMaxAccBW(float value)  { m_maxAccelerationBackward = value; };
    inline void setMaxDecBW(float value)  { m_maxDecelerationBackward = value; };

    virtual void getConfiguration(QCBOREncodeContext &EncodeCtx);


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
    float m_maxAttainableSpeed;
    float m_timeToVmax;
    float m_timeFromVmaxToZero;

    float m_damplingFactor;
};

#endif /* SRC_ACCELERATIONDECELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_ */
