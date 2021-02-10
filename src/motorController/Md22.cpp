#include "Md22.h"
#include <ch.h>
#include <hal.h>
#include "util/asservMath.h"

constexpr uint8_t md22Address = 0xB0 >> 1; // MD22 address (All switches to ON) 0x10110000 =>1011000 0x58
constexpr uint8_t modeReg = 0x00;
constexpr uint8_t motor1Reg = 0x01;
constexpr uint8_t motor2Reg = 0x02;
constexpr uint8_t accReg = 0x03;

constexpr uint8_t controlMode = 0x01; // Wanted value for mode register. Ie: -128 (full reverse)   0 (stop)   127 (full forward).

 Md22::Md22(I2cPinInit *i2cPins, bool is1motorRight, bool invertMotorRight, bool invertMotorLeft, uint32_t i2cFrequency) :
        MotorController()
{
    m_i2cPinConf = *i2cPins;
    m_invertMotorRight = invertMotorRight;
    m_invertMotorLeft = invertMotorLeft;
    m_is1motorRight = is1motorRight;
    m_lastRightConsign = 0;
    m_lastLeftConsign = 0;

    chDbgAssert(i2cFrequency <= 400000, "Md22: i2cFrequency shall be lower than 400khz \r\n");

    if (i2cFrequency <= 100000)
        m_i2cconfig = { OPMODE_I2C, i2cFrequency, STD_DUTY_CYCLE };
    else if (i2cFrequency > 100000)
        m_i2cconfig = { OPMODE_I2C, i2cFrequency, FAST_DUTY_CYCLE_2 };
}

void Md22::init()
{
    // Enable I2C SDA & SCL pin
    // External pullups with correct resistance value shall be used !
    // see : http://wiki.chibios.org/dokuwiki/doku.php?id=chibios:community:guides:i2c_trouble_shooting
    palSetPadMode(m_i2cPinConf.GPIObaseSCL, m_i2cPinConf.pinNumberSCL,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(m_i2cPinConf.GPIObaseSDA, m_i2cPinConf.pinNumberSDA,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    //chThdSleepMilliseconds(500);
    i2cStart(&I2CD1, &m_i2cconfig);

    // When the stm32 and the Md22 are powered on at the same time,
    //   it seems that the MD22 could take much longer to boot...
    // So wait a few ms !
    chThdSleepMilliseconds(100);

    i2cAcquireBus (&I2CD1);

    sysinterval_t tmo = TIME_MS2I(4);

    // Set mode
    uint8_t cmd[] = { modeReg, controlMode };
    msg_t msg = i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);

    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout ERROR NOK\r\n");

    // Set acceleration
    cmd[0] = accReg;
    cmd[1] = 0;
    msg = i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout ERROR NOK\r\n");

    // Set motor speed to zero
    cmd[0] = motor1Reg;
    cmd[1] = 0;
    msg = i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout ERROR NOK\r\n");

    cmd[0] = motor2Reg;
    cmd[1] = 0;
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
    uint8_t reg ;
    if (m_is1motorRight)
        reg = motor2Reg;
    else
        reg = motor1Reg;

    uint8_t cmd[] = { reg, (uint8_t) md22SpeedConsign };
    i2cAcquireBus (&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_MS2I(1));
    i2cReleaseBus(&I2CD1);
    m_lastLeftConsign = md22SpeedConsign;
}

void Md22::setMotorRightSpeed(float percentage)
{
    percentage = limit(percentage, -100.0, 100.0);

    if (m_invertMotorRight)
        percentage = -percentage;

    int8_t md22SpeedConsign = (int8_t) fmap(percentage, -100.0, 100.0, -128.0, 127.0);
    uint8_t reg;
    if (m_is1motorRight)
        reg = motor1Reg;
    else
        reg = motor2Reg;

    uint8_t cmd[] = { reg, (uint8_t) md22SpeedConsign };
    i2cAcquireBus (&I2CD1);
    i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_MS2I(1));
    i2cReleaseBus(&I2CD1);
    m_lastRightConsign = md22SpeedConsign;
}

