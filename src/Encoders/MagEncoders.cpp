#include "MagEncoders.h"
#include <ch.h>
#include <hal.h>
#include "util/asservMath.h"
#include "ams_as5048b.h"

MagEncoders::MagEncoders(bool is1EncoderRight, bool invertEncoderRight, bool invertEncoderLeft) :
        Encoders(), m_mysensor1(AS5048B_ADDR(0, 0)), m_mysensor2(AS5048B_ADDR(1, 0))
{

    I2cPinInit encodersI2cPinsConf_SCL_SDA = { GPIOB, 10, GPIOB, 3 };
    m_i2cPinConf = encodersI2cPinsConf_SCL_SDA;
    m_i2cconfig = {OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2};

    m_invertEncoderR = invertEncoderRight;
    m_invertEncoderL = invertEncoderLeft;
    m_encoderRSum = 0;
    m_encoderLSum = 0;
    m_encoder1Previous = 0;
    m_encoder2Previous = 0;
    m_is1EncoderRight = is1EncoderRight;

}

MagEncoders::~MagEncoders()
{
}

void MagEncoders::init()
{
    // Enable I2C SDA & SCL pin
    // External pullups with correct resistance value shall be used !
    // see : http://wiki.chibios.org/dokuwiki/doku.php?id=chibios:community:guides:i2c_trouble_shooting
    palSetPadMode(m_i2cPinConf.GPIObaseSCL, m_i2cPinConf.pinNumberSCL,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(m_i2cPinConf.GPIObaseSDA, m_i2cPinConf.pinNumberSDA,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    i2cStart(&I2CD2, &m_i2cconfig);
    //wait 200ms
    chThdSleepMilliseconds(200);

    m_mysensor1.begin();
    m_mysensor2.begin();
}

void MagEncoders::start()
{

}

void MagEncoders::stop()
{

}

void MagEncoders::getValues(int16_t *encoderRight, int16_t *encoderLeft)
{
    int16_t encoder1 = m_mysensor1.angleR(U_RAW, true);
    int16_t encoder2 = m_mysensor2.angleR(U_RAW, true);

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

}

void MagEncoders::getEncodersTotalCount(int32_t *encoderRight, int32_t *encoderLeft)
{
    *encoderRight = m_encoderRSum;
    *encoderLeft = m_encoderLSum;
}
void MagEncoders::getValuesStatus(int16_t *encoderRight, int16_t *encoderLeft, uint8_t *agcR, uint8_t *agcL,
        uint8_t *diagR, uint8_t *diagL, uint16_t *magR, uint16_t *magL, uint16_t *rawR, uint16_t *rawL)
{
    uint8_t data1[6];
    uint8_t data2[6];

    m_mysensor1.readRegs( AS5048B_GAIN_REG, 6, data1);
    m_mysensor2.readRegs( AS5048B_GAIN_REG, 6, data2);


    int16_t encoder1 = m_mysensor1.angleR(U_RAW, true);
    int16_t encoder2 = m_mysensor2.angleR(U_RAW, true);

    if (m_is1EncoderRight) {
        *encoderRight = encoder1 - m_encoder1Previous;
        *agcR = data1[0];
        *diagR = data1[1];
        *magR = ((uint16_t) (data1[2]) << 6) + (data1[3] & 0x3F);
        *rawR = ((uint16_t) (data1[4]) << 6) + (data1[5] & 0x3F);

        *encoderLeft = encoder2 - m_encoder2Previous;
        *agcL = data2[0];
        *diagL = data2[1];
        *magL = ((uint16_t) (data2[2]) << 6) + (data2[3] & 0x3F);
        *rawL = ((uint16_t) (data2[4]) << 6) + (data2[5] & 0x3F);

    } else {
        *encoderRight = encoder2 - m_encoder2Previous;
        *agcR = data2[0];
        *diagR = data2[1];
        *magR = ((uint16_t) (data2[2]) << 6) + (data2[3] & 0x3F);
        *rawR = ((uint16_t) (data2[4]) << 6) + (data2[5] & 0x3F);

        *encoderLeft = encoder1 - m_encoder1Previous;
        *agcL = data1[0];
        *diagL = data1[1];
        *magL = ((uint16_t) (data1[2]) << 6) + (data1[3] & 0x3F);
        *rawL = ((uint16_t) (data1[4]) << 6) + (data1[5] & 0x3F);
    }

    if (m_invertEncoderR)
        *encoderRight = -*encoderRight;
    if (m_invertEncoderL)
        *encoderLeft = -*encoderLeft;

    m_encoderRSum += *encoderRight;
    m_encoderLSum += *encoderLeft;

    m_encoder1Previous = encoder1;
    m_encoder2Previous = encoder2;
}

