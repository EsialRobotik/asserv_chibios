#include "MagEncoders.h"
#include "ch.h"
#include "hal.h"

MagEncoders::MagEncoders(bool is1EncoderRight, bool invertEncoderR, bool invertEncoderL) :
        Encoders()
{
    /*
    m_invertEncoderR = invertEncoderR;
    m_invertEncoderL = invertEncoderL;
    m_encoderRSum = 0;
    m_encoderLSum = 0;
    m_encoder1Previous = 0;
    m_encoder2Previous = 0;
    m_is1EncoderRight = is1EncoderRight;
    */

}

MagEncoders::~MagEncoders()
{
}

void MagEncoders::init()
{
    /*
    // Encoder 1
    palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(2)); //TIM3_chan2
    palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(2)); //TIM3_chan1
    qeiStart(&QEID3, &qeicfg1);

    // Encoder 2
    palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(1)); //TIM2_chan1
    palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(1)); //TIM2_chan2
    qeiStart(&QEID2, &qeicfg2);
    */
}

void MagEncoders::start()
{

}

void MagEncoders::stop()
{

}

void MagEncoders::getValues(int16_t *encoderRight, int16_t *encoderLeft)
{
/*
    int16_t encoder2 = qeiGetCount(&QEID2);
    int16_t encoder1 = qeiGetCount(&QEID3);

    if (m_is1EncoderRight) {
        *encoderRight = encoder1 - m_encoder1Previous;
        *encoderLeft = encoder2 - m_encoder2Previous;
    } else {
        *encoderRight = encoder2 - m_encoder2Previous;
        *encoderLeft = encoder1 - m_encoder1Previous;
    }

    if (m_invertEncoderR)
        *encoderRight = -*encoderRight;
    if (m_invertEncoderL)
        *encoderLeft = -*encoderLeft;

    m_encoderRSum += *encoderRight;
    m_encoderLSum += *encoderLeft;

    m_encoder1Previous = encoder1;
    m_encoder2Previous = encoder2;
    */
}

void MagEncoders::getEncodersTotalCount(int32_t *encoderRight, int32_t *encoderLeft)
{
    /*
    *encoderRight = m_encoderRSum;
    *encoderLeft = m_encoderLSum;
    */
}
