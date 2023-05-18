#ifndef SRC_BLOCKINGDETECTOR_SPEEDERRORBLOCKINGDETECTOR_H_
#define SRC_BLOCKINGDETECTOR_SPEEDERRORBLOCKINGDETECTOR_H_

#include <cstdint>

#include "blockingDetector/BlockingDetector.h"


class SpeedController;

class SpeedErrorBlockingDetector : public BlockingDetector
{
    public:
        /**
         * Unfortunately this class doesn't really work ! It's almost impossible to tune the parameters.
         *  Just keep this implementation for education purpose.
         */
        explicit SpeedErrorBlockingDetector(float dt,  SpeedController& rightSpeedController, SpeedController& leftSpeedController, float movingIntegralDuration, float movingIntegralErrorThreshold);
        virtual ~SpeedErrorBlockingDetector();

        virtual bool isBlocked() const;
        virtual void update();

        virtual void reset() { } // TODO

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

#endif /* SRC_BLOCKINGDETECTOR_SPEEDERRORBLOCKINGDETECTOR_H_ */
