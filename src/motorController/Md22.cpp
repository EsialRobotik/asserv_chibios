#include "Md22.h"
#include <ch.h>
#include <hal.h>
#include "util/asservMath.h"

static const I2CConfig i2cconfig = { OPMODE_I2C, 100000, STD_DUTY_CYCLE };

constexpr uint8_t md22Address = 0xB0 >> 1; // MD22 address (All switches to ON) 0x10110000 =>1011000 0x58
constexpr uint8_t modeReg = 0x00;
constexpr uint8_t motor1Reg = 0x01;
constexpr uint8_t motor2Reg = 0x02;
constexpr uint8_t accReg = 0x03;

constexpr uint8_t controlMode = 0x01; // Wanted value for mode register. Ie: -128 (full reverse)   0 (stop)   127 (full forward).

Md22::Md22(bool is1motorRight, bool invertMotorRight, bool invertMotorLeft) : MotorController()
{
    m_invertMotorRight = invertMotorRight;
    m_invertMotorLeft = invertMotorLeft;
    m_is1motorRight = is1motorRight;
}

void Md22::init()
{
    sysinterval_t tmo = TIME_MS2I(4);
    // Enable I2C SDA & SCL pin
    // External pullups with correct resistance value shall be used !
    // see : http://wiki.chibios.org/dokuwiki/doku.php?id=chibios:community:guides:i2c_trouble_shooting
    palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);


    // TODO : make this configurable
//    palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); //SCL
//    palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); //SDA

    i2cStart(&I2CD1, &i2cconfig);

    i2cAcquireBus (&I2CD1);

    // Set mode
    uint8_t cmd[] = { modeReg, controlMode };
    msg_t msg = i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);

    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout ERROR NOK\r\n");

    // Set acceleration
    cmd[0] = accReg;
    cmd[0] = 0;
    msg = i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout ERROR NOK\r\n");


    // Set motor speed to zero
    cmd[0] = motor1Reg;
    cmd[0] = 0;
    msg = i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout ERROR NOK\r\n");


    cmd[0] = motor2Reg;
    cmd[0] = 0;
    msg = i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout ERROR NOK\r\n");

    i2cReleaseBus(&I2CD1);
}


void Md22::setMotorLeftSpeed(float percentage)
{
    percentage = limit(percentage, -100.0, 100.0);

    if (m_invertMotorLeft)
        percentage = -percentage;

    int8_t md22SpeedConsign = (int8_t) fmap(percentage, -100.0, 100.0, -128.0, 127.0);
    uint8_t reg = 0;
    if (m_is1motorRight)
        reg = motor2Reg;
     else
        reg = motor1Reg;

    uint8_t cmd[] = { reg, (uint8_t) md22SpeedConsign };
    i2cAcquireBus (&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_MS2I(1));
    i2cReleaseBus(&I2CD1);
}

void Md22::setMotorRightSpeed(float percentage)
{
    percentage = limit(percentage, -100.0, 100.0);

    if (m_invertMotorRight)
        percentage = -percentage;

    int8_t md22SpeedConsign = (int8_t) fmap(percentage, -100.0, 100.0, -128.0, 127.0);
    uint8_t reg = 0;
    if (m_is1motorRight)
        reg = motor1Reg;
    else
        reg = motor2Reg;

    uint8_t cmd[] = { reg, (uint8_t) md22SpeedConsign };
    i2cAcquireBus (&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_MS2I(1));
    i2cReleaseBus(&I2CD1);
}

