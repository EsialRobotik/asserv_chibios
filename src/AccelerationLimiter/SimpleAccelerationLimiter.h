#ifndef SRC_ACCELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_
#define SRC_ACCELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_

#include "AbstractAccelerationLimiter.h"
#include "sampleStream/configuration/ConfigurationInterface.h"


class SimpleAccelerationLimiter : public AbstractAccelerationLimiter, public Configuration
{

    /*
     * Simple acceleration limiter is ... Simple !
     *
     *   When the robot is accelerating
     *   ( determined by the upper part of the class, ie: AbstractAccelerationLimiter),
     *    the next output speed consign is limited by m_maxAcceleration.
     *
     *  Output speed will be a trapezoidal shape velocity
     */

public:
    explicit SimpleAccelerationLimiter(float maxAcceleration);
    virtual ~SimpleAccelerationLimiter(){};

    void setMaxAcceleration(float maxAcceleration);
    inline float getMaxAcceleration() const { return m_maxAcceleration; };

    /*!
     * \brief Echelle 0..100% appliquee dynamiquement sur le delta dans
     *        limitOutput. Ne modifie pas m_maxAcceleration. Permet de
     *        ralentir l'angle (rotation) en plus de la distance via
     *        setAccDecPercent (cmd CBOR 18).
     */
    void setSpeedPercent(float percent) override;
    inline float getSpeedPercent() const { return m_speedScale * 100.0f; };

    virtual void getConfiguration(QCBOREncodeContext &EncodeCtx);

    virtual void applyConfiguration(QCBORDecodeContext &decodeCtx);

private:
    virtual float limitOutput(float dt, float targetSpeed, float previousOutput, float currentSpeed);

    float m_maxAcceleration;
    float m_speedScale;     // facteur 0..1 applique dans limitOutput (1.0 = 100% nominal)
};

#endif /* SRC_ACCELERATIONLIMITER_SIMPLEACCELERATIONLIMITER_H_ */
