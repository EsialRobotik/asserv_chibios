#ifndef SRC_ACCELERATIONLIMITER_H_
#define SRC_ACCELERATIONLIMITER_H_

class AccelerationDecelerationLimiterInterface
{
public:
    virtual ~AccelerationDecelerationLimiterInterface()
    {
    }

    virtual float limitAcceleration(float dt, float targetSpeed, float currentSpeed, float positionGoal, float positionError) = 0;

    virtual void enable() = 0;
    virtual void disable() = 0;
    virtual void reset() = 0;

    /*!
     * \brief Echelle de vitesse 0..100% appliquee a l'acc/dec.
     *        Default no-op : seuls les limiters qui implementent cette
     *        methode (ex: AdvancedAccelerationLimiter) reagissent.
     *        Permet a AsservMain::setSpeedPercent de notifier polymorphi-
     *        quement tous les limiters sans connaitre leur type concret.
     */
    virtual void setSpeedPercent(float /* percent */) {}
};

#endif /* SRC_ACCELERATIONLIMITER_H_ */
