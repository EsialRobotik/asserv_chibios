#ifndef USBSTREAM_SRC_DATASTREAMTYPE_H_
#define USBSTREAM_SRC_DATASTREAMTYPE_H_

#include <stdint.h>
#include <cmath>
#include "ch.h"
#include "USBStreamMacros.h"


class USBStream
{
public:
    static void init();
    static inline USBStream* instance()
    {
        return s_instance;
    }

    void* sendCurrentStream();

    void sendConfig(uint8_t *configBuffer, uint8_t size);


    void USBStreamHandleConnection_lowerpriothread();

    void releaseBuffer();
    void getFullBuffer(void** ptr, uint32_t* size);

    /*
     * DIRTY HACK !!
     *   as uart over usb doesn't seems to like zeros,
     *       replace them by NaN that will be replaced by zeros in Plotjuggler
     */
    inline static void setValue(void *ptr, float value)
    {
        float *ptrFlt = (float*) ptr;
        if (value == 0.0)
            *ptrFlt = NAN;
        else
            *ptrFlt = value;
    };


    GENERATE_USB_STREAM_HEADER(
        Timestamp,                              timestamp,
        SpeedGoalRight,                         speed/right/goal,
        SpeedEstimatedRight,                    speed/right/current,
        SpeedOutputRight,                       speed/right/outputConsign ,
        SpeedIntegratedOutputRight,             speed/right/output_consign_integrated,
        SpeedKpRight,                           speed/right/Kp,
        SpeedKiRight,                           speed/right/Ki,
        SpeedGoalLeft,                          speed/left/goal,
        SpeedEstimatedLeft,                     speed/left/current,
        SpeedOutputLeft,                        speed/left/outputConsign,
        SpeedIntegratedOutputLeft,              speed/left/output_consign_integrated,
        SpeedKpLeft,                            speed/left/Kp,
        SpeedKiLeft,                            speed/left/Ki,
        AngleGoal,                              angle_regulator/goal,
        AngleAccumulator,                       angle_regulator/accumulator,
        AngleOutput,                            angle_regulator/output,
        AngleOutputLimited,                     angle_regulator/limited_output,
        DistGoal,                               distance_regulator/goal,
        DistAccumulator,                        distance_regulator/accumulator,
        DistOutput,                             distance_regulator/output,
        DistOutputLimited,                      distance_regulator/limited_output,
        RawEncoderDeltaRight,                   raw_encoder/right,
        RawEncoderDeltaLeft,                    raw_encoder/left,
        OdoX,                                   odometry/X,
        OdoY,                                   odometry/Y,
        OdoTheta,                               odometry/theta,
        XGoal,                                  commandManager/X,
        YGoal,                                  commandManager/Y,
        DistanceLimiterVelocityAtDecTime,       accDec/distance/VelocityAtDecTime,
        DistanceLimiterVelocityCompensation,    accDec/distance/VelocityCompensation,
        DistanceLimiterVelocityCompensated,     accDec/distance/VelocityCompensated,
        DistanceLimiterOutput,                  accDec/distance/OutputSpeedConsign,
        DistanceLimiterTimeFromVmaxToZero,      accDec/distance/TimeFromVmaxToZero,
        DistanceLimiterTargetSpeed,             accDec/distance/targetSpeed,
        DistanceLimiterTimeToVMax,              accDec/distance/TimeToVMax,
        DistanceLimiterMaxAttainableSpeed,      accDec/distance/MaxAttainableSpeed,
        DistanceLimitercurrentSpeed,            accDec/distance/currentSpeed,
        MovingIntegralError,                    blockingDetector/movingIntegralError,
        MovingIntegralErrorThreshold,           blockingDetector/movingIntegralErrorThreshold);


private:
    USBStream();
    virtual ~USBStream()
    {
    }
    ;

    void getEmptyBuffer();
    void sendFullBuffer();

    static USBStream* s_instance;

    UsbStreamSample *m_currentPtr;
    UsbStreamSample m_currentStruct;
    uint32_t m_timestamp;
    uint32_t m_bufferSize;
    mutex_t m_sample_sending_mutex;
};

#endif /* USBSTREAM_SRC_DATASTREAMTYPE_H_ */
