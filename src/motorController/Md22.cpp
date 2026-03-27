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

 Md22::Md22(I2cPinInit *i2cPins, I2CDriver *i2cDriver, bool is1motorRight, bool invertMotorRight, bool invertMotorLeft, uint32_t i2cFrequency) :
        MotorController()
{
    m_i2cPinConf = *i2cPins;
    m_i2cDriver = i2cDriver;
    m_i2cFrequency = i2cFrequency;
    m_invertMotorRight = invertMotorRight;
    m_invertMotorLeft = invertMotorLeft;
    m_is1motorRight = is1motorRight;
    m_lastRightConsign = 0;
    m_lastLeftConsign = 0;
    m_rightMotorPercentage = 0;
    m_leftMotorPercentage = 0;
    

    chDbgAssert(i2cFrequency <= 400000, "Md22: i2cFrequency shall be lower than 400khz \r\n");
}

void Md22::init()
{
    I2CConfig m_i2cconfig;

    #if defined(STM32F4)
    if (m_i2cFrequency <= 100000)
        m_i2cconfig = { OPMODE_I2C, m_i2cFrequency, STD_DUTY_CYCLE };
    else if (m_i2cFrequency > 100000)
        m_i2cconfig = { OPMODE_I2C, m_i2cFrequency, FAST_DUTY_CYCLE_2 };
    #elif defined(STM32G4)
        m_i2cconfig = {0x20B17DB6, 0, 0}; // 1ist value is timingr register, this specific value is computed by cubeMX for an Nucleo G431KB. Need to be adapted if used on another CPU.
    #else
        #error Unknown target in MD22 implementation, add your  !
    #endif

    // Enable I2C SDA & SCL pin
    // External pullups with correct resistance value shall be used !
    // see : http://wiki.chibios.org/dokuwiki/doku.php?id=chibios:community:guides:i2c_trouble_shooting
    palSetPadMode(m_i2cPinConf.GPIObaseSCL, m_i2cPinConf.pinNumberSCL,
            PAL_MODE_ALTERNATE(m_i2cPinConf.SCLPinAlternate) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(m_i2cPinConf.GPIObaseSDA, m_i2cPinConf.pinNumberSDA,
            PAL_MODE_ALTERNATE(m_i2cPinConf.SDAPinAlternate) | PAL_STM32_OTYPE_OPENDRAIN);

    i2cStart(m_i2cDriver, &m_i2cconfig);

    // When the stm32 and the Md22 are powered on at the same time,
    //   it seems that the MD22 could take much longer to boot...
    // So wait a few ms !
    chThdSleepMilliseconds(100);


    sysinterval_t tmo = TIME_MS2I(10);
    msg_t msg = 0;
    uint8_t cmd[] = {0,0};

    i2cAcquireBus (m_i2cDriver);


    // Set mode
    cmd[0] = modeReg;
    cmd[1] = controlMode;
    msg = i2cMasterTransmitTimeout(m_i2cDriver, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout Set mode ERROR NOK\r\n");

    // Set acceleration
    cmd[0] = accReg;
    cmd[1] = 0;
    msg = i2cMasterTransmitTimeout(m_i2cDriver, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout Set acceleration ERROR NOK\r\n");

    // Set motor speed to zero
    cmd[0] = motor1Reg;
    cmd[1] = 0;
    msg = i2cMasterTransmitTimeout(m_i2cDriver, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout motor1Reg ERROR NOK\r\n");

    cmd[0] = motor2Reg;
    cmd[1] = 0;
    msg = i2cMasterTransmitTimeout(m_i2cDriver, md22Address, cmd, sizeof(cmd), NULL, 0, tmo);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout motor2Reg ERROR NOK\r\n");

    i2cReleaseBus(m_i2cDriver);
}

void Md22::setMotorLeftSpeed(float percentage)
{
    m_leftMotorPercentage = limit(percentage, -100.0, 100.0);

    if (m_invertMotorLeft)
        m_leftMotorPercentage = -m_leftMotorPercentage;

    int8_t md22SpeedConsign = (int8_t) fmap(m_leftMotorPercentage, -100.0, 100.0, -128.0, 127.0);
    uint8_t reg ;
    if (m_is1motorRight)
        reg = motor2Reg;
    else
        reg = motor1Reg;

    uint8_t cmd[] = { reg, (uint8_t) md22SpeedConsign };
    i2cAcquireBus (m_i2cDriver);
    i2cMasterTransmitTimeout(m_i2cDriver, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_MS2I(1));
    i2cReleaseBus(m_i2cDriver);
    m_lastLeftConsign = md22SpeedConsign;
}

void Md22::setMotorRightSpeed(float percentage)
{
    m_rightMotorPercentage = limit(percentage, -100.0, 100.0);

    if (m_invertMotorRight)
        m_rightMotorPercentage = -m_rightMotorPercentage;

    int8_t md22SpeedConsign = (int8_t) fmap(m_rightMotorPercentage, -100.0, 100.0, -128.0, 127.0);
    uint8_t reg;
    if (m_is1motorRight)
        reg = motor1Reg;
    else
        reg = motor2Reg;

    uint8_t cmd[] = { reg, (uint8_t) md22SpeedConsign };
    i2cAcquireBus (m_i2cDriver);
    i2cMasterTransmitTimeout(m_i2cDriver, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_MS2I(1));
    i2cReleaseBus(m_i2cDriver);
    m_lastRightConsign = md22SpeedConsign;
}

float Md22::getMotorRightSpeedNonInverted() const
{
    if (m_invertMotorRight)
        return -m_rightMotorPercentage;
    else
        return m_rightMotorPercentage;
}

float Md22::getMotorLeftSpeedNonInverted() const
{
    if (m_invertMotorLeft)
        return -m_leftMotorPercentage;
    else
        return m_leftMotorPercentage;
}

