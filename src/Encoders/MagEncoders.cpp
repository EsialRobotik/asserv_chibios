#include "MagEncoders.h"
#include <ch.h>
#include <hal.h>
#include "util/asservMath.h"
#include "ams_as5048b.h"
//#include <chprintf.h>

//extern BaseSequentialStream *outputStream;

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
    //wait 20ms
    chThdSleepMilliseconds(20);

    m_mysensor1.begin();
    m_mysensor2.begin();

    //verification
    uint8_t agc = 0;
    uint8_t diag = 0;
    uint16_t mag = 0;
    uint16_t raw = 0;
    m_mysensor1.getAllData(&agc, &diag, &mag, &raw);
    chDbgAssert((diag == 1), "init() m_mysensor1 MagEncoders - getAllData (diag == 1) NOK\r\n");
    chDbgAssert((agc >= 30 && agc <= 70),
            "init() m_mysensor1 MagEncoders - getAllData (agc >= 30 && agc <= 70) NOK\r\n");

    m_mysensor2.getAllData(&agc, &diag, &mag, &raw);
    chDbgAssert((diag == 1), "init() m_mysensor2 MagEncoders - getAllData (diag == 1) NOK\r\n");
    chDbgAssert((agc >= 30 && agc <= 70),
            "init() m_mysensor2 MagEncoders - getAllData (agc >= 30 && agc <= 70) NOK\r\n");

    //chprintf(outputStream,"MagEncoders::init() done;\r\n");
}

void MagEncoders::start()
{
    m_encoder1Previous = (int16_t) ((m_mysensor1.angleR(U_RAW, true) - 8192.0) * 4.0);
    m_encoder2Previous = (int16_t) ((m_mysensor2.angleR(U_RAW, true) - 8192.0) * 4.0);
    m_encoderRSum = 0;
    m_encoderLSum = 0;

    //chprintf(outputStream,"MagEncoders::start() done; %d %d\r\n",m_encoder1Previous, m_encoder2Previous);
}

void MagEncoders::stop()
{
    start();
}

void MagEncoders::getValues(float *deltaEncoderRight, float *deltaEncoderLeft)
{
    //chprintf(outputStream,"MagEncoders::getValues() done; %d %d\r\n", m_encoder1Previous, m_encoder2Previous);
    //utilisation du depassement d'un int16
    //[0;16383] -8192 * 4 = [-32768;32764]
    float encoder1 = (m_mysensor1.angleR(U_RAW, true) - 8192.0) * 4.0;
    float encoder2 = (m_mysensor2.angleR(U_RAW, true) - 8192.0) * 4.0;

    if (m_is1EncoderRight) {
        *deltaEncoderRight = encoder1 - m_encoder1Previous;
        *deltaEncoderLeft = encoder2 - m_encoder2Previous;
    } else {
        *deltaEncoderRight = encoder2 - m_encoder2Previous;
        *deltaEncoderLeft = encoder1 - m_encoder1Previous;
    }

    if (m_invertEncoderR)
        *deltaEncoderRight = -*deltaEncoderRight;
    if (m_invertEncoderL)
        *deltaEncoderLeft = -*deltaEncoderLeft;

    *deltaEncoderRight = (int16_t)(*deltaEncoderRight / 4.0);
    *deltaEncoderLeft = (int16_t)(*deltaEncoderLeft / 4.0);

    m_encoderRSum += (int32_t)(*deltaEncoderRight);
    m_encoderLSum += (int32_t)(*deltaEncoderLeft);

    m_encoder1Previous = encoder1;
    m_encoder2Previous = encoder2;
}

void MagEncoders::getEncodersTotalCount(int32_t *encoderRight, int32_t *encoderLeft)
{
    *encoderRight = m_encoderRSum;
    *encoderLeft = m_encoderLSum;
}

void MagEncoders::getValuesStatus(uint16_t *rawR, uint16_t *rawL, uint8_t *agcR, uint8_t *agcL, uint8_t *diagR,
        uint8_t *diagL, uint16_t *magR, uint16_t *magL)
{
    m_mysensor1.getAllData(agcR, diagR, magR, rawR);
    m_mysensor2.getAllData(agcL, diagL, magL, rawL);
}
