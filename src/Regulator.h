#ifndef REGULATOR_H_
#define REGULATOR_H_

#include <cstdint>
#include "sampleStream/configuration/ConfigurationInterface.h"

class Regulator : public Configuration
{
public:
    explicit Regulator(float Kp, float max_output);
    virtual ~Regulator(){};
    
    void updateFeedback(float feedback);
    float updateOutput(float goal);

    float getAccumulator() const
    {
        return m_accumulator;
    };

    void setGain(float Kp)
    {
        m_Kp = Kp;
    }

    float getGain() const
    {
        return m_Kp;
    };

    void reset()
    {
        m_accumulator = 0;
    };

    float getError() const
    {
        return m_error;
    };

    float getOutput() const
    {
        return m_output;
    };

    void enable()
    {
        m_enabled = true;
    };

    void disable()
    {
        m_enabled = false;
    };

    void setMaxOutput(float max_output)
    {
        m_maxOutput = max_output;
    };
    
    virtual void getConfiguration(QCBOREncodeContext &EncodeCtx);

    virtual void applyConfiguration(QCBORDecodeContext &decodeCtx);
    

private:
    double m_accumulator;
    float m_Kp;
    float m_error;
    float m_output;
    float m_maxOutput;
    float m_enabled;
};

#endif /* REGULATOR_H_ */
