#ifndef SRC_SPEEDCONTROLLER_H_
#define SRC_SPEEDCONTROLLER_H_

#include <cstdint>

class SpeedController
{
public:
    explicit SpeedController(float speedKp, float speedKi, float outputLimit, float maxInputSpeed, float measureFrequency);
    virtual ~SpeedController(){};

    virtual float update(float actualSpeed);

    virtual void setGains(float Kp, float Ki);

    void setSpeedGoal(float speed);
    float getSpeedGoal() const
    {
        return m_speedGoal;
    }

    float getIntegratedOutput() const
    {
        return m_integratedOutput;
    }

    float getCurrentKp() const
    {
        return m_speedKp;
    }

    float getCurrentKi() const
    {
        return m_speedKi;
    }

    void resetIntegral()
    {
        m_integratedOutput = 0;
    }

    void setMaxOutputLimitRatio(float outputLimitPercentage)
    {
    	m_outputLimitRatio = outputLimitPercentage/100.0;
    }

    void setMaxInputLimit(float max)
    {
        m_inputLimit = max;
    }

protected:
    float m_speedKp;
    float m_speedKi;


private:
    float m_speedGoal;
    float m_integratedOutput;

    float m_outputLimit;
    float m_outputLimitRatio;
    float m_inputLimit;

    float m_measureFrequency;
};

#endif /* SRC_SPEEDCONTROLLER_H_ */
