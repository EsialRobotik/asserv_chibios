#ifndef SRC_SLOPE_FILTER_H_
#define SRC_SLOPE_FILTER_H_

class SlopeFilter
{
    public:
        explicit SlopeFilter(float maxSlope);
        virtual ~SlopeFilter(){};

        float filter(float dt, float value);
        void setSlope(float slope);
        void reset(){m_lastOutput = 0;};

    private:
        float m_maxSlope;
        float m_lastOutput;

        float constrain(float value, float low, float high);
};

#endif /* SRC_SLOPE_FILTER_H_ */
