#include "Md22.h"
#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include "util/asservMath.h"

static const I2CConfig i2cconfig = { OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2 //FAST_DUTY_CYCLE_2 //STD_DUTY_CYCLE ne fct pas car 400000
        };

constexpr uint8_t md22Address = 0xB0 >> 1; // MD22 address (All switches to ON) 0x10110000 =>1011000 0x58
constexpr uint8_t modeReg = 0x00;
constexpr uint8_t leftMotorReg = 0x01;
constexpr uint8_t rightMotorReg = 0x02;
constexpr uint8_t accReg = 0x03;

constexpr uint8_t controlMode = 0x01; // Wanted value for mode register. Ie: -128 (full reverse)   0 (stop)   127 (full forward).

//extern BaseSequentialStream *outputStream;

Md22::Md22(bool is1motorRight, bool invertMotorR, bool invertMotorL)
{

    m_invertMotorR = invertMotorR;
    m_invertMotorL = invertMotorL;
    m_is1motorRight = is1motorRight;
}

void Md22::init()
{

    sysinterval_t tmo = TIME_MS2I(4);
    // Enable I2C SDA & SCL pin
    // External pullups with correct resistance value shall be used !
    // see : http://wiki.chibios.org/dokuwiki/doku.php?id=chibios:community:guides:i2c_trouble_shooting
    //palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    //palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); //SCL
    palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); //SDA

    //chprintf(outputStream," init\r\n");

    //chprintf(outputStream,"ibefore\r\n");
    i2cStart(&I2CD1, &i2cconfig);

    //chprintf(outputStream,"iafter\r\n");

    i2cAcquireBus (&I2CD1);

    // Set mode
    uint8_t cmd[] = { modeReg, controlMode };
    msg_t msg = i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);

    i2c_lld_get_errors(&I2CD1);

    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout ERROR NOK\r\n");
    if (msg != MSG_OK) {
        // What to do ?
        //chprintf(outputStream, " i2cMasterTransmitTimeout ERROR NOK\r\n");
    }

    // Set acceleration
    cmd[0] = accReg;
    cmd[0] = 0;
    i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);

    // Set motor speed to zero
    cmd[0] = leftMotorReg;
    cmd[0] = 0;
    i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);

    cmd[0] = rightMotorReg;
    cmd[0] = 0;
    i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);

    i2cReleaseBus(&I2CD1);
}

void Md22::setMotorLSpeed(float percentage)
{
    if (m_invertMotorL)
        percentage = -percentage;

    int8_t md22SpeedConsign = (int8_t) fmap(percentage, -100.0, 100.0, -128.0, 127.0);
    uint8_t reg = 0;
    if (m_is1motorRight) {
        reg = leftMotorReg;
    } else {
        reg = rightMotorReg;
    }
    uint8_t cmd[] = { reg, (uint8_t) md22SpeedConsign };
    i2cAcquireBus (&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_INFINITE);
    i2cReleaseBus(&I2CD1);
}

void Md22::setMotorRSpeed(float percentage)
{
    if (m_invertMotorR)
        percentage = -percentage;

    int8_t md22SpeedConsign = (int8_t) fmap(percentage, -100.0, 100.0, -128.0, 127.0);
    uint8_t reg = 0;
    if (m_is1motorRight) {
        reg = rightMotorReg;
    } else {
        reg = leftMotorReg;
    }
    uint8_t cmd[] = { reg, (uint8_t) md22SpeedConsign };
    i2cAcquireBus (&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_INFINITE);
    i2cReleaseBus(&I2CD1);
}

Md22::~Md22()
{
}

