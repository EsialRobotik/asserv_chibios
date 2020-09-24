#ifndef SRC_SLOPE_FILTER_H_
#define SRC_SLOPE_FILTER_H_

class SlopeFilter
{
public:
    explicit SlopeFilter(float maxAcceleration, float maxAccelerationLowSpeed, float lowSpeecThreshold);
    virtual ~SlopeFilter() {};

    float filter(float dt, float targetSpeed, float currentSpeed);
    void setSlope(float slope);
    void reset() { m_lastOutput = 0; };

    void enable();
    void disable();

private:
    float m_maxAcceleration;
    float m_maxAccelerationLowSpeed;
    float m_lowSpeedThreshold;
    float m_maxDeceleration;
    float m_lastOutput;
    bool  m_enabled;

    float constrain(float value, float low, float high);
};

#endif /* SRC_SLOPE_FILTER_H_ */
