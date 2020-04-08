#include "MagEncoders.h"
#include <ch.h>
#include <hal.h>
#include "util/asservMath.h"
#include "ams_as5048b.h"


MagEncoders::MagEncoders(I2cPinInit pins, bool is1EncoderRight, bool invertEncoderRight, bool invertEncoderLeft) :
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

    m_i2cconfig = {OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2};

    m_mysensor = new AMS_AS5048B(AS5048B_ADDR(0,0));

}

MagEncoders::~MagEncoders()
{
}

void MagEncoders::init()
{
    sysinterval_t tmo = TIME_MS2I(4);

    // Enable I2C SDA & SCL pin
    // External pullups with correct resistance value shall be used !
    // see : http://wiki.chibios.org/dokuwiki/doku.php?id=chibios:community:guides:i2c_trouble_shooting
    palSetPadMode(m_i2cPinConf.GPIObaseSCL, m_i2cPinConf.pinNumberSCL,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(m_i2cPinConf.GPIObaseSDA, m_i2cPinConf.pinNumberSDA,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    i2cStart(&I2CD2, &m_i2cconfig);

    i2cAcquireBus (&I2CD2);

    // Set mode
    //    uint8_t cmd[] = { MD22_MODEREG, MD22_CONTROLMODE };
    //    msg_t msg = i2cMasterTransmitTimeout(&I2CD1, MD22_ADDRESS, cmd, sizeof(cmd), NULL, 0, tmo);

    i2cReleaseBus(&I2CD2);
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
