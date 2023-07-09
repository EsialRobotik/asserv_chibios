#include "ch.h"
#include "EncoderSimulation.h"
#include "hal.h"


EncoderSimuation::EncoderSimuation() :
        Encoders()
{
    m_encoderR = 0;
    m_encoderL = 0;
}

EncoderSimuation::~EncoderSimuation()
{
}


void EncoderSimuation::getValues(float *deltaEncoderRight, float *deltaEncoderLeft)
{
    *deltaEncoderRight = m_encoderR;
    *deltaEncoderLeft = m_encoderL;
}
