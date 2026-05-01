#include "AdvancedAccelerationLimiter.h"
#include <cmath>


AdvancedAccelerationLimiter::AdvancedAccelerationLimiter(float maxAcceleration, float minAcceleration, float highSpeedThreshold) :
AbstractAccelerationLimiter()
{
    m_maxAcceleration = maxAcceleration;
    m_minAcceleration = minAcceleration;
    m_HighSpeedThreshold = highSpeedThreshold;
    m_speedScale = 1.0f;     // 100% nominal au boot
}

float AdvancedAccelerationLimiter::limitOutput(float dt, float targetSpeed, float previousOutput, float currentSpeed)
{
    float change = targetSpeed - previousOutput;
    float maxDelta;

    if( fabs(currentSpeed) >= m_HighSpeedThreshold)
       maxDelta = dt * m_maxAcceleration;
   else
       maxDelta = dt * (m_minAcceleration + fabs(currentSpeed)*(m_maxAcceleration-m_minAcceleration)/m_HighSpeedThreshold);

    maxDelta *= m_speedScale;    // applique l'echelle 0..1 (set_speed_percent)
    return constrain(change, -maxDelta, maxDelta);
}

void AdvancedAccelerationLimiter::setSpeedPercent(float percent)
{
    if (percent < 1.0f)   percent = 1.0f;     // safety : evite acc=0 qui figerait le robot
    if (percent > 100.0f) percent = 100.0f;
    m_speedScale = percent / 100.0f;
}

void AdvancedAccelerationLimiter::setMaxAcceleration(float maxAcceleration)
{
    m_maxAcceleration = maxAcceleration;
}

void AdvancedAccelerationLimiter::setMinAcceleration(float minAcceleration)
{
    m_minAcceleration = minAcceleration;
}

void AdvancedAccelerationLimiter::setHighSpeedThreshold(float highSpeedThreshold)
{
    m_HighSpeedThreshold = highSpeedThreshold;
}


void AdvancedAccelerationLimiter::getConfiguration(QCBOREncodeContext &EncodeCtx)
{
    UsefulBufC name = {.ptr = "adv_acc_limiter", .len = strlen("adv_acc_limiter")};
    QCBOREncode_AddTextToMapSZ (&EncodeCtx, "name", name);
    QCBOREncode_AddFloatToMapSZ(&EncodeCtx, "max_acc", m_maxAcceleration);
    QCBOREncode_AddFloatToMapSZ(&EncodeCtx, "min_acc", m_minAcceleration);
    QCBOREncode_AddFloatToMapSZ(&EncodeCtx, "highspeed_threshold", m_HighSpeedThreshold);
}

void AdvancedAccelerationLimiter::applyConfiguration(QCBORDecodeContext &decodeCtx)
{
    double max_acc;
	QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "max_acc", &max_acc);
    double min_acc;
	QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "min_acc", &min_acc);
    double highspeed_threshold;
	QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "highspeed_threshold", &highspeed_threshold);

    if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
    {
        setMaxAcceleration(max_acc);
        setMinAcceleration(min_acc);
        setHighSpeedThreshold(highspeed_threshold);
    }
}