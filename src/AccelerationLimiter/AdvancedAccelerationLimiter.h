#ifndef SRC_ACCELERATIONLIMITER_ADVANCEDACCELERATIONLIMITER_H_
#define SRC_ACCELERATIONLIMITER_ADVANCEDACCELERATIONLIMITER_H_

#include "AbstractAccelerationLimiter.h"
#include "sampleStream/configuration/ConfigurationInterface.h"


class AdvancedAccelerationLimiter : public AbstractAccelerationLimiter , public Configuration
{

    /*
     * Advanced acceleration limiter is a bit more complex!
     *
     *   When the robot is accelerating
     *   ( determined by the upper part of the class, ie: AbstractAccelerationLimiter),
     *    the next output speed consign is limited considering the current speed.
     *
     *  As the robot can accelerate more when it's already moving than when it's almost stationary.
     *  So we determine a max acceleration that will be used when the robot speed reach highSpeedThreshold.
     *  Below this speed, a linear interpolation between minAcceleration and maxAcceleration will be use !
     *
     *  This implementation will limit the jerk of the robot.
     */

public:
    explicit AdvancedAccelerationLimiter(float maxAcceleration, float minAcceleration, float highSpeedThreshold);
    virtual ~AdvancedAccelerationLimiter(){};

    void setMaxAcceleration(float maxAcceleration);
    void setMinAcceleration(float minAcceleration);
    void setHighSpeedThreshold(float highSpeedThreshold);

    /*!
     * \brief Echelle de 0 a 100% appliquee dynamiquement sur les deltas
     *        d'acceleration calcules dans limitOutput. Permet de demander
     *        au robot de rouler "doucement" sans modifier les valeurs de
     *        reference m_maxAcceleration / m_minAcceleration.
     * \param percent valeur entre 1 et 100 (clampe dans la methode)
     */
    void setSpeedPercent(float percent) override;
    inline float getSpeedPercent() const { return m_speedScale * 100.0f; };

    inline float getMaxAcceleration() const { return m_maxAcceleration; };
    inline float getMinAcceleration() const { return m_minAcceleration; };
    inline float getHighSpeedThreshold() const{ return m_HighSpeedThreshold; };

    virtual void getConfiguration(QCBOREncodeContext &EncodeCtx);
    virtual void applyConfiguration(QCBORDecodeContext &decodeCtx);

private:
    virtual float limitOutput(float dt, float targetSpeed, float previousOutput, float currentSpeed);

    float m_maxAcceleration;
    float m_minAcceleration;
    float m_HighSpeedThreshold;
    float m_speedScale;     // facteur 0.0..1.0 applique dans limitOutput (1.0 = 100% nominal)
};

#endif /* SRC_ACCELERATIONLIMITER_ADVANCEDACCELERATIONLIMITER_H_ */
