#ifndef SRC_SPEEDCONTROLLER_H_
#define SRC_SPEEDCONTROLLER_H_

#define NB_PI_SUBSET (3)

class SpeedController
{

    /*
     * Le controlleur de vitesse est en fait un PI adaptatif. C'est à dire que les valeurs de Kp et Ki changent en fonction de la vitesse actuelle
     *  Il y a NB_PI_SUBSET set différents, qui sont choisis en fonction de setSpeedRange.
     *   Voir la fonction computeGains pour le détail du calcul.
     */

public:
    explicit SpeedController(float speedKpSet[NB_PI_SUBSET], float speedKiSet[NB_PI_SUBSET],
            float setSpeedRange[NB_PI_SUBSET], float outputLimit, float maxInputSpeed, float measureFrequency);
    virtual ~SpeedController()
    {
    }
    ;

    float update(float actualSpeed);

    void setGains(float Kp, float Ki);

    void setSpeedGoal(float speed);
    float getSpeedGoal()
    {
        return m_speedGoal;
    }
    ;
    float getIntegratedOutput()
    {
        return m_integratedOutput;
    }
    ;

    void resetIntegral()
    {
        m_integratedOutput = 0;
    }
    ;

private:
    float m_speedGoal;
    float m_integratedOutput;

    float m_speedKp;
    float m_speedKi;

    float m_setSpeedRange[NB_PI_SUBSET];
    float m_speedKpSet[NB_PI_SUBSET];
    float m_speedKiSet[NB_PI_SUBSET];

    float m_outputLimit;
    float m_inputLimit;

    float m_measureFrequency;

    void computeGains(float actualSpeed);
};

#endif /* SRC_SPEEDCONTROLLER_H_ */
