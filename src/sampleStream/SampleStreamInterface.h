#ifndef SRC_SAMPLE_STREAM_ITF_H_
#define SRC_SAMPLE_STREAM_ITF_H_


#include <cstdint>
#include <cmath>
#include "SampleStreamMacros.h"
#include "msgpack11.hpp"

using namespace msgpack11;


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

//    GENERATE_USB_STREAM_HEADER(
//        Timestamp,                              timestamp,
//        SpeedGoalRight,                         speed/right/goal,
//        SpeedEstimatedRight,                    speed/right/current,
//        SpeedOutputRight,                       speed/right/outputConsign ,
//        SpeedIntegratedOutputRight,             speed/right/output_consign_integrated,
//        SpeedKpRight,                           speed/right/Kp,
//        SpeedKiRight,                           speed/right/Ki,
//        SpeedGoalLeft,                          speed/left/goal,
//        SpeedEstimatedLeft,                     speed/left/current,
//        SpeedOutputLeft,                        speed/left/outputConsign,
//        SpeedIntegratedOutputLeft,              speed/left/output_consign_integrated,
//        SpeedKpLeft,                            speed/left/Kp,
//        SpeedKiLeft,                            speed/left/Ki,
//        AngleGoal,                              angle_regulator/goal,
//        AngleAccumulator,                       angle_regulator/accumulator,
//        AngleOutput,                            angle_regulator/output,
//        AngleOutputLimited,                     angle_regulator/limited_output,
//        DistGoal,                               distance_regulator/goal,
//        DistAccumulator,                        distance_regulator/accumulator,
//        DistOutput,                             distance_regulator/output,
//        DistOutputLimited,                      distance_regulator/limited_output,
//        RawEncoderDeltaRight,                   raw_encoder/right,
//        RawEncoderDeltaLeft,                    raw_encoder/left,
//        OdoX,                                   odometry/X,
//        OdoY,                                   odometry/Y,
//        OdoTheta,                               odometry/theta,
//        XGoal,                                  commandManager/X,
//        YGoal,                                  commandManager/Y,
//        AlignOnly,                             commandManager/AlignOnly,
//        DistanceLimiterVelocityAtDecTime,       accDec/distance/VelocityAtDecTime,
//        DistanceLimiterVelocityCompensation,    accDec/distance/VelocityCompensation,
//        DistanceLimiterVelocityCompensated,     accDec/distance/VelocityCompensated,
//        DistanceLimiterOutput,                  accDec/distance/OutputSpeedConsign,
//        DistanceLimiterTimeFromVmaxToZero,      accDec/distance/TimeFromVmaxToZero,
//        DistanceLimiterTargetSpeed,             accDec/distance/targetSpeed,
//        DistanceLimiterTimeToVMax,              accDec/distance/TimeToVMax,
//        DistanceLimiterMaxAttainableSpeed,      accDec/distance/MaxAttainableSpeed,
//        DistanceLimitercurrentSpeed,            accDec/distance/currentSpeed,
//        BlockingDuration,                       blockingDetector/blockingDuration,
//        BlockingDetected,                       blockingDetector/blockingDetected);

    MsgPack::object my_msgpack = {
        {"timestamp", 0.0f},
        {"speed/right/goal", 0.0f},
        {"speed/right/current", 0.0f},
        {"speed/right/outputConsign", 0.0f},
        {"speed/right/output_consign_integrated", 0.0f},
        {"speed/right/Kp", 0.0f},
        {"speed/right/Ki", 0.0},
        {"speed/left/goal", 0.0},
        {"speed/left/current", 0.0},
        {"speed/left/outputConsign", 0.0},
        {"speed/left/output_consign_integrated", 0.0},
        {"speed/left/Kp", 0.0},
        {"speed/left/Ki", 0.0},
        {"angle_regulator/goal", 0.0},
        {"angle_regulator/accumulator", 0.0},
        {"angle_regulator/output", 0.0},
        {"angle_regulator/limited_output", 0.0},
        {"distance_regulator/goal", 0.0},
        {"distance_regulator/accumulator", 0.0},
        {"distance_regulator/output", 0.0},
        {"distance_regulator/limited_output", 0.0},
        {"raw_encoder/right", 0.0},
        {"raw_encoder/left", 0.0},
        {"odometry/X", 0.0},
        {"odometry/Y", 0.0},
        {"odometry/theta", 0.0},
        {"commandManager/X", 0.0},
        {"commandManager/Y", 0.0},
        {"ommandManager/AlignOnly", 0.0},
        {"accDec/distance/VelocityAtDecTime", 0.0},
        {"accDec/distance/VelocityCompensation", 0.0},
        {"accDec/distance/VelocityCompensated", 0.0},
        {"accDec/distance/OutputSpeedConsign", 0.0},
        {"accDec/distance/TimeFromVmaxToZero", 0.0},
        {"accDec/distance/targetSpeed", 0.0},
        {"accDec/distance/TimeToVMax", 0.0},
        {"accDec/distance/MaxAttainableSpeed", 0.0},
        {"accDec/distance/currentSpeed", 0.0},
        {"blockingDetector/blockingDuration", 0.0},
        {"blockingDetector/blockingDetected)", 0.0}
    };

    inline void setTimestamp(float val) {my_msgpack["timestamp"] = val;};
    inline void setSpeedGoalRight(float val) {my_msgpack["speed/right/goal"] = val;};
    inline void setSpeedEstimatedRight(float val) {my_msgpack["speed/right/current"] = val;};
    inline void setSpeedOutputRight(float val) {my_msgpack["speed/right/outputConsign"] = val;};
    inline void setSpeedIntegratedOutputRight(float val) {my_msgpack["speed/right/output_consign_integrated"] = val;};
    inline void setSpeedKpRight(float val) {my_msgpack["speed/right/Kp"] = val;};
    inline void setSpeedKiRight(float val) {my_msgpack["speed/right/Ki"] = val;};
    inline void setSpeedGoalLeft(float val) {my_msgpack["speed/left/goal"] = val;};
    inline void setSpeedEstimatedLeft(float val) {my_msgpack["speed/left/current"] = val;};
    inline void setSpeedOutputLeft(float val) {my_msgpack["speed/left/outputConsign"] = val;};
    inline void setSpeedIntegratedOutputLeft(float val) {my_msgpack["speed/left/output_consign_integrated"] = val;};
    inline void setSpeedKpLeft(float val) {my_msgpack["speed/left/Kp"] = val;};
    inline void setSpeedKiLeft(float val) {my_msgpack["speed/left/Ki"] = val;};
    inline void setAngleGoal(float val) {my_msgpack["angle_regulator/goal"] = val;};
    inline void setAngleAccumulator(float val) {my_msgpack["angle_regulator/accumulator"] = val;};
    inline void setAngleOutput(float val) {my_msgpack["angle_regulator/output"] = val;};
    inline void setAngleOutputLimited(float val) {my_msgpack["angle_regulator/limited_output"] = val;};
    inline void setDistGoal(float val) {my_msgpack["distance_regulator/goal"] = val;};
    inline void setDistAccumulator(float val) {my_msgpack["distance_regulator/accumulator"] = val;};
    inline void setDistOutput(float val) {my_msgpack["distance_regulator/output"] = val;};
    inline void setDistOutputLimited(float val) {my_msgpack["distance_regulator/limited_output"] = val;};
    inline void setRawEncoderDeltaRight(float val) {my_msgpack["raw_encoder/right"] = val;};
    inline void setRawEncoderDeltaLeft(float val) {my_msgpack["raw_encoder/left"] = val;};
    inline void setOdoX(float val) {my_msgpack["odometry/X"] = val;};
    inline void setOdoY(float val) {my_msgpack["odometry/Y"] = val;};
    inline void setOdoTheta(float val) {my_msgpack["odometry/theta"] = val;};
    inline void setXGoal(float val) {my_msgpack["commandManager/X"] = val;};
    inline void setYGoal(float val) {my_msgpack["commandManager/Y"] = val;};
    inline void setAlignOnly(float val) {my_msgpack["ommandManager/AlignOnly"] = val;};
    inline void setDistanceLimiterVelocityAtDecTime(float val) {my_msgpack["accDec/distance/VelocityAtDecTime"] = val;};
    inline void setDistanceLimiterVelocityCompensation(float val) {my_msgpack["accDec/distance/VelocityCompensation"] = val;};
    inline void setDistanceLimiterVelocityCompensated(float val) {my_msgpack["accDec/distance/VelocityCompensated"] = val;};
    inline void setDistanceLimiterOutput(float val) {my_msgpack["accDec/distance/OutputSpeedConsign"] = val;};
    inline void setDistanceLimiterTimeFromVmaxToZero(float val) {my_msgpack["accDec/distance/TimeFromVmaxToZero"] = val;};
    inline void setDistanceLimiterTargetSpeed(float val) {my_msgpack["accDec/distance/targetSpeed"] = val;};
    inline void setDistanceLimiterTimeToVMax(float val) {my_msgpack["accDec/distance/TimeToVMax"] = val;};
    inline void setDistanceLimiterMaxAttainableSpeed(float val) {my_msgpack["accDec/distance/MaxAttainableSpeed"] = val;};
    inline void setDistanceLimitercurrentSpeed(float val) {my_msgpack["accDec/distance/currentSpeed"] = val;};
    inline void setBlockingDuration(float val) {my_msgpack["blockingDetector/blockingDuration"] = val;};
    inline void setBlockingDetected(float val) {my_msgpack["blockingDetector/blockingDetected"] = val;};


private:

    static inline SampleStream* s_instance = nullptr;

//protected:
//    UsbStreamSample *m_currentPtr;
//    UsbStreamSample m_currentStruct;

};


#endif /* SRC_SAMPLE_STREAM_ITF_H_ */
