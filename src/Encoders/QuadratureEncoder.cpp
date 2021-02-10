#include "QuadratureEncoder.h"
#include "ch.h"
#include "hal.h"

__extension__ static QEIConfig qeicfg1 = {
		.mode = QEI_MODE_QUADRATURE,
		.resolution = QEI_BOTH_EDGES,
		.dirinv = QEI_DIRINV_FALSE,
		.overflow = QEI_OVERFLOW_WRAP,
		.min = 0,
		.max = 0,
		.notify_cb = nullptr,
		.overflow_cb = nullptr
};

static QEIConfig qeicfg2 = {
		.mode = QEI_MODE_QUADRATURE,
		.resolution = QEI_BOTH_EDGES,
		.dirinv = QEI_DIRINV_FALSE,
		.overflow = QEI_OVERFLOW_WRAP,
		.min = 0,
		.max = 0,
		.notify_cb = nullptr,
		.overflow_cb = nullptr
};

QuadratureEncoder::QuadratureEncoder(GpioPinInit *gpioPins, bool is1EncoderRight, bool invertEncoderR, bool invertEncoderL) :
        Encoders()
{
    m_gpioPinConf = *gpioPins;
    m_invertEncoderR = invertEncoderR;
    m_invertEncoderL = invertEncoderL;
    m_encoderRSum = 0;
    m_encoderLSum = 0;
    m_encoder1Previous = 0;
    m_encoder2Previous = 0;
    m_is1EncoderRight = is1EncoderRight;
    m_encoderLGain = 1;
    m_encoderRGain = 1;
}

QuadratureEncoder::~QuadratureEncoder()
{
}

void QuadratureEncoder::init()
{
    // Encoder 1
    palSetPadMode(m_gpioPinConf.GPIObaseE1ch2, m_gpioPinConf.pinNumberE1ch2, PAL_MODE_ALTERNATE(2)); //TIM3_chan2
    palSetPadMode(m_gpioPinConf.GPIObaseE1ch1, m_gpioPinConf.pinNumberE1ch1, PAL_MODE_ALTERNATE(2)); //TIM3_chan1
    qeiStart(&QEID3, &qeicfg1);
    // Encoder 2
    palSetPadMode(m_gpioPinConf.GPIObaseE2ch2, m_gpioPinConf.pinNumberE2ch2, PAL_MODE_ALTERNATE(1)); //TIM2_chan2
    palSetPadMode(m_gpioPinConf.GPIObaseE2ch1, m_gpioPinConf.pinNumberE2ch1, PAL_MODE_ALTERNATE(1)); //TIM2_chan1
    qeiStart(&QEID2, &qeicfg2);
//    // Encoder 1
//    palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(2)); //TIM3_chan2
//    palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(2)); //TIM3_chan1
//    qeiStart(&QEID3, &qeicfg1);
//
//    // Encoder 2
//    //palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(1)); //TIM2_chan2
//    //palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(1)); //TIM2_chan1
//    palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(1)); //TIM2_chan1
//    palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(1)); //TIM2_chan2
//    qeiStart(&QEID2, &qeicfg2);
}

void QuadratureEncoder::start()
{
    qeiEnable (&QEID3);
    qeiEnable (&QEID2);
}

void QuadratureEncoder::stop()
{
    qeiDisable (&QEID3);
    qeiDisable (&QEID2);
}

void QuadratureEncoder::getValues(float *deltaEncoderRight, float *deltaEncoderLeft)
{
    int16_t encoder2 = qeiGetCount(&QEID2);
    int16_t encoder1 = qeiGetCount(&QEID3);

    int16_t deltaRight;
    int16_t deltaLeft;

    if (m_is1EncoderRight)
    {
        deltaRight = encoder1 - m_encoder1Previous;
        deltaLeft = encoder2 - m_encoder2Previous;
    }
    else
    {
        deltaRight = encoder2 - m_encoder2Previous;
        deltaLeft = encoder1 - m_encoder1Previous;
    }

    if (m_invertEncoderR)
        deltaRight = -deltaRight;
    if (m_invertEncoderL)
        deltaLeft = -deltaLeft;

    m_encoderRSum += deltaRight;
    m_encoderLSum += deltaLeft;

    *deltaEncoderRight = float(deltaRight) * m_encoderRGain;
    *deltaEncoderLeft = float(deltaLeft) * m_encoderLGain;

    m_encoder1Previous = encoder1;
    m_encoder2Previous = encoder2;
}
