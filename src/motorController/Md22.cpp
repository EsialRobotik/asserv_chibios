#include "Md22.h"
#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "util/asservMath.h"

constexpr uint8_t md22Address = 0xB0 >> 1; // MD22 address (All switches to ON) 0x10110000 =>1011000 0x58
constexpr uint8_t modeReg = 0x00;
constexpr uint8_t motor1Reg = 0x01;
constexpr uint8_t motor2Reg = 0x02;
constexpr uint8_t accReg = 0x03;

constexpr uint8_t controlMode = 0x01; // Wanted value for mode register. Ie: -128 (full reverse)   0 (stop)   127 (full forward).

extern BaseSequentialStream *outputStream;

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
    chThdSleepMilliseconds(400);

    // Enable I2C SDA & SCL pin
    // External pullups with correct resistance value shall be used !
    // see : http://wiki.chibios.org/dokuwiki/doku.php?id=chibios:community:guides:i2c_trouble_shooting
    palSetPadMode(m_i2cPinConf.GPIObaseSCL, m_i2cPinConf.pinNumberSCL,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(m_i2cPinConf.GPIObaseSDA, m_i2cPinConf.pinNumberSDA,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);


    i2cStart(&I2CD1, &m_i2cconfig);

    // When the stm32 and the Md22 are powered on at the same time,
    //   it seems that the MD22 could take much longer to boot...
    // So wait a few ms !
    //chThdSleepMilliseconds(2);//100



    sysinterval_t tmo = TIME_MS2I(4);
    msg_t msg = 0;
    uint8_t cmd[] = {0,0};

    i2cAcquireBus (&I2CD1);
    // Set mode
    cmd[0] = modeReg;
    cmd[1] = controlMode;
    msg = i2cMasterTransmitTimeoutTimes(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo, 5);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout Set mode ERROR NOK\r\n");

    // Set acceleration
    cmd[0] = accReg;
    cmd[1] = 0;
    msg = i2cMasterTransmitTimeoutTimes(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo, 5);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout Set acceleration ERROR NOK\r\n");

    // Set motor speed to zero
    cmd[0] = motor1Reg;
    cmd[1] = 0;
    msg = i2cMasterTransmitTimeoutTimes(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo, 5);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout motor1Reg ERROR NOK\r\n");

    cmd[0] = motor2Reg;
    cmd[1] = 0;
    msg = i2cMasterTransmitTimeoutTimes(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, tmo, 5);
    chDbgAssert(msg == MSG_OK, "Config MD22 - i2cMasterTransmitTimeout motor2Reg ERROR NOK\r\n");

    i2cReleaseBus(&I2CD1);
}

msg_t Md22::i2cMasterTransmitTimeoutTimes(I2CDriver *i2cp,
                               i2caddr_t addr,
                               const uint8_t *txbuf,
                               size_t txbytes,
                               uint8_t *rxbuf,
                               size_t rxbytes,
                               sysinterval_t timeout, int times)
{

    msg_t r;
   for(int i = 0; i <= times ; i++)
   {
       if (i2cp->state != I2C_READY)
       {
           i2cStart(i2cp, i2cp->config);
           chThdSleepMilliseconds(2);
       }
       r = i2cMasterTransmitTimeout(i2cp, addr, txbuf, txbytes, rxbuf, rxbytes, timeout);
       if (r == MSG_OK)
           return r;
       else
       {
           chprintf(outputStream,"...Md22::i2cMasterTransmitTimeoutTimes try... %d  \r\n", i);

           i2cReleaseBus(&I2CD1);
           chThdSleepMilliseconds(2);
           i2cAcquireBus(&I2CD1);
       }
   }

   return r;
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
    i2cMasterTransmitTimeoutTimes(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_MS2I(1), 15);
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
    i2cMasterTransmitTimeoutTimes(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_MS2I(1), 15);
    i2cReleaseBus(&I2CD1);
    m_lastRightConsign = md22SpeedConsign;
}

