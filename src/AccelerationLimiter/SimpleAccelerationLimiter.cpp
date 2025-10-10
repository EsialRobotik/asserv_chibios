#include "SimpleAccelerationLimiter.h"

SimpleAccelerationLimiter::SimpleAccelerationLimiter(float maxAcceleration) : AbstractAccelerationLimiter()
{
    m_maxAcceleration = maxAcceleration;
}

float SimpleAccelerationLimiter::limitOutput(float dt, float targetSpeed, float previousOutput, float)
{
    float change = targetSpeed - previousOutput;
    float maxDelta = dt * m_maxAcceleration;

    return constrain(change, -maxDelta, maxDelta);
}


void SimpleAccelerationLimiter::setMaxAcceleration(float maxAcceleration)
{
    m_maxAcceleration = maxAcceleration;
}


void SimpleAccelerationLimiter::getConfiguration(QCBOREncodeContext &EncodeCtx)
{
    UsefulBufC name = {.ptr = "acc_limiter", .len = strlen("acc_limiter")};
    QCBOREncode_AddTextToMapSZ (&EncodeCtx, "name", name);
    QCBOREncode_AddFloatToMapSZ(&EncodeCtx, "max_acc", m_maxAcceleration);
}


void SimpleAccelerationLimiter::applyConfiguration(QCBORDecodeContext &decodeCtx)
{
    double max_acc;
	QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "max_acc", &max_acc);

    if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
    {
        setMaxAcceleration(max_acc);
    }
}