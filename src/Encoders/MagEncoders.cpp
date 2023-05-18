#include "MagEncoders.h"
#include <ch.h>
#include <hal.h>
#include "util/asservMath.h"
#include "ams_as5048b.h"
#include <chprintf.h>
#include "hal_streams.h"
#include "util/debug.h"


MagEncoders::MagEncoders(I2cPinInit *i2cPins, bool is1EncoderRight, bool invertEncoderRight, bool invertEncoderLeft,
        uint32_t i2cFrequency) :
        Encoders(), m_mysensor1(AS5048B_ADDR(1, 1)), m_mysensor2(AS5048B_ADDR(0, 1))
{
    m_i2cPinConf = *i2cPins;
    if (i2cFrequency <= 100000)
        m_i2cconfig = { OPMODE_I2C, i2cFrequency, STD_DUTY_CYCLE };
    else if (i2cFrequency > 100000)
        m_i2cconfig = { OPMODE_I2C, i2cFrequency, FAST_DUTY_CYCLE_2 };

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
    //debug1("MagEncoders::init()... \r\n");

    // Enable I2C SDA & SCL pin
    // External pullups with correct resistance value shall be used !
    // see : http://wiki.chibios.org/dokuwiki/doku.php?id=chibios:community:guides:i2c_trouble_shooting
    palSetPadMode(m_i2cPinConf.GPIObaseSCL, m_i2cPinConf.pinNumberSCL,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);debug1("MagEncoders::palSetPadMode() OK \r\n");

    palSetPadMode(m_i2cPinConf.GPIObaseSDA, m_i2cPinConf.pinNumberSDA,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);debug1("MagEncoders::palset OK \r\n");

    //     * Keep this hacky part from PMRobotix
    //     *     because they tries to get rid of interference with software.....
    //     * TODO: check if this is REALLY necessary
    for (int i = 0; i < 10; i++) {
        if (I2CD2.state != I2C_READY) {
            i2cStart(&I2CD2, &m_i2cconfig);
            chThdSleepMilliseconds(2);
        }
    }
    //i2cStart(&I2CD2, &m_i2cconfig);

    debug1("MagEncoders::i2cStart() OK \r\n");

    //wait 20ms => better 40ms
    chThdSleepMilliseconds(40);

    //verification
    uint8_t agc = 0;
    uint8_t diag = 0;
    uint16_t mag = 0;
    uint16_t raw = 0;

    int connect1 = m_mysensor1.begin();
    if (connect1 < 0) {
        debug2("MagEncoders::init() - m_mysensor1 NOT CONNECTED %d\r\n",connect1);

    } else {

        m_mysensor1.getAllData(&agc, &diag, &mag, &raw);
        chprintf(outputStream, "1.0x%02x agc=%d diag=%d mag=%d raw=%d\r\n", m_mysensor1.chipAddress(), agc, diag, mag,
                raw);
        chThdSleepMilliseconds(20);
        chDbgAssert((diag == 1), "init() m_mysensor1 MagEncoders - getAllData (diag != 1) NOK\r\n");
        if (!(agc >= 30 && agc <= 150)) {
            chprintf(outputStream, "ERROR 1.0x%02x agc=%d NOK (agc >= 30 && agc <= 90)\r\n", m_mysensor1.chipAddress(), agc);
            chThdSleepMilliseconds(20);
            chDbgAssert((agc >= 30 && agc <= 150),
                    "init() m_mysensor1 MagEncoders - getAllData (agc >= 30 && agc <= 90) NOK\r\n");
        }
    }

    int connect2 = m_mysensor2.begin();
    if (connect2 < 0) {
        debug2("MagEncoders::init() - m_mysensor2 NOT CONNECTED %d\r\n",connect2);
        chThdSleepMilliseconds(20);
        chDbgAssert((connect2 == 0), "MagEncoders::init() - m_mysensor2 NOT CONNECTED\r\n");
    } else {

        m_mysensor2.getAllData(&agc, &diag, &mag, &raw);
        chprintf(outputStream, "2.0x%02x agc=%d diag=%d mag=%d raw=%d\r\n", m_mysensor2.chipAddress(), agc, diag, mag,
                raw);
        chThdSleepMilliseconds(20);

        chDbgAssert((diag == 1), "init() m_mysensor2 MagEncoders - getAllData (diag == 1) NOK\r\n");
        if (!(agc >= 30 && agc <= 150)) {
            chprintf(outputStream, "ERROR 2.0x%02x agc=%d NOK (agc >= 30 && agc <= 80)\r\n", m_mysensor2.chipAddress(), agc);
            chThdSleepMilliseconds(20);
            chDbgAssert((agc >= 30 && agc <= 150),
                    "init() m_mysensor2 MagEncoders - getAllData (agc >= 30 && agc <= 80) NOK\r\n");
        }
    }
    if ((connect1 < 0 || connect2 < 0))
    {
        chThdSleepMilliseconds(20);
        //chDbgAssert((connect1 == 0 || connect2 == 0), "MagEncoders::init() - m_mysensor1 or 2 NOT CONNECTED\r\n");


        //Allumer des led d'error
        blink_error(50);
    }
    //debug
//    float encoder1 = (m_mysensor1.angleR(U_RAW, true) );
//    float encoder2 = (m_mysensor2.angleR(U_RAW, true) );
    //chprintf(outputStream,"MagEncoders::init() %f  %f;\r\n", encoder1,encoder2);

    //chprintf(outputStream,"MagEncoders::init() done;\r\n");
}

void MagEncoders::start()
{
    m_encoder1Previous = ((m_mysensor1.angleR(U_RAW, true) - 8192.0) * 4.0);
    m_encoder2Previous = ((m_mysensor2.angleR(U_RAW, true) - 8192.0) * 4.0);
    m_encoderRSum = 0;
    m_encoderLSum = 0;
}

void MagEncoders::stop()
{
    start();
}

void MagEncoders::getValues(float *deltaEncoderRightFiltered, float *deltaEncoderLeftFiltered)
{
    float deltaEncoderRight = 0.0;
    float deltaEncoderLeft = 0.0;
    //utilisation du depassement d'un int16
    //[0;16383] -8192 * 4 = [-32768;32764]

    uint16_t raw1 = 0;
    uint16_t raw2 = 0;
    uint8_t agc1 = 0;
    uint8_t agc2 = 0;
    uint8_t diag1 = 0;
    uint8_t diag2 = 0;
    uint16_t mag1 = 0;
    uint16_t mag2 = 0;
    getValuesStatus(&raw1, &raw2, &agc1, &agc2, &diag1, &diag2, &mag1, &mag2);
    float encoder1 = ((float) (raw1) - 8192.0) * 4.0;
    float encoder2 = ((float) (raw2) - 8192.0) * 4.0;

    if (m_is1EncoderRight) {
        deltaEncoderRight = encoder1 - m_encoder1Previous;
        deltaEncoderLeft = encoder2 - m_encoder2Previous;
    } else {
        deltaEncoderRight = encoder2 - m_encoder2Previous;
        deltaEncoderLeft = encoder1 - m_encoder1Previous;
    }

    if (m_invertEncoderR)
        deltaEncoderRight = -deltaEncoderRight;
    if (m_invertEncoderL)
        deltaEncoderLeft = -deltaEncoderLeft;

    deltaEncoderRight = ((int16_t) (deltaEncoderRight)) / 4.0;
    deltaEncoderLeft = ((int16_t) (deltaEncoderLeft)) / 4.0;

    m_encoderRSum += (int32_t) (deltaEncoderRight);
    m_encoderLSum += (int32_t) (deltaEncoderLeft);

    m_encoder1Previous = encoder1;
    m_encoder2Previous = encoder2;

    *deltaEncoderRightFiltered = (deltaEncoderRight);
    *deltaEncoderLeftFiltered = (deltaEncoderLeft);
}

void MagEncoders::getEncodersTotalCount(int32_t *encoderRight, int32_t *encoderLeft)
{
    *encoderRight = m_encoderRSum;
    *encoderLeft = m_encoderLSum;
}

void MagEncoders::getValuesStatus(uint16_t *raw1, uint16_t *raw2, uint8_t *agc1, uint8_t *agc2, uint8_t *diag1,
        uint8_t *diag2, uint16_t *mag1, uint16_t *mag2)
{
    m_mysensor1.getAllData(agc1, diag1, mag1, raw1);
    m_mysensor2.getAllData(agc2, diag2, mag2, raw2);
}
