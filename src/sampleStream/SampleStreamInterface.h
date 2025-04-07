#ifndef SRC_SAMPLE_STREAM_ITF_H_
#define SRC_SAMPLE_STREAM_ITF_H_


#include <cstdint>
#include <cmath>
#include "SampleStreamMacros.h"

class SampleStream
{
public:
    virtual ~SampleStream(){}

    static SampleStream* instance()
    {
        return s_instance;
    }

    static void setInstance(SampleStream* instance)
    {
        s_instance = instance;
    }

    virtual void* sendCurrentStream() = 0;


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


    /*
     * This macro will generate accessor for each value
     *      and generate a name that will be sent to plotjuggler.
     *
     * For example :
     * GENERATE_USB_STREAM_HEADER(
     *  SpeedGoalRight,               speed/right/goal,
     *  SpeedIntegratedOutputRight,   speed/right/output_consign_integrated)
     *
     *  will generation 2 function : void setSpeedGoalRight(float value) and void SpeedIntegratedOutputRight(float value)
     *
     *  and 2 string "speed/right/goal" and "speed/right/output_consign_integrated)" will be sent to plotjuggler.
     */

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
        BlockingDuration,                       blockingDetector/blockingDuration,
        BlockingDetected,                       blockingDetector/blockingDetected
        // DecelerationStartPos,                   accDec/distance/decelerationStartPosition,
        // posAtNextTick,                          accDec/distance/posAtNextTick,
        // NextSpeedConsign,                       accDec/distance/nextSpeedConsign,
        // outputGoal,                             accDec/distance/outputgoal,
        // CurrentSpeed,                           accDec/distance/currentSpeedRamp,
        // IsAcc,                                  accDec/distance/IsAcc,
        // speedStep,                                  accDec/distance/speedStep
        );


private:

    static inline SampleStream* s_instance = nullptr;

protected:
    UsbStreamSample *m_currentPtr;
    UsbStreamSample m_currentStruct;

};


#endif /* SRC_SAMPLE_STREAM_ITF_H_ */
