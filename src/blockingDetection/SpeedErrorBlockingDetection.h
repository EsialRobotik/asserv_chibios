#ifndef SRC_BLOCKINGDETECTION_SPEEDERRORBLOCKINGDETECTION_H_
#define SRC_BLOCKINGDETECTION_SPEEDERRORBLOCKINGDETECTION_H_

#include "BlockingDetection.h"
#include <cstdint>


class SpeedController;

class SpeedErrorBlockingDetection : BlockingDetection
{
    public:
        explicit SpeedErrorBlockingDetection(float dt,  SpeedController& rightSpeedController, SpeedController& leftSpeedController, float movingIntegralDuration, float movingIntegralErrorThreshold);
        virtual ~SpeedErrorBlockingDetection();

        virtual bool isBlocked() ;

    private:
        float m_dt;
        SpeedController& m_rightSpeedController;
        SpeedController& m_leftSpeedController;
        uint32_t m_nbValues;
        uint32_t m_currentIdx;
        float *m_errorValues;
        float m_movingIntegralError;
        float m_movingIntegralErrorThreshold;
};

#endif /* SRC_BLOCKINGDETECTION_SPEEDERRORBLOCKINGDETECTION_H_ */
