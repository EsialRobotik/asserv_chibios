#include "Md22.h"
#include "ch.h"
#include "hal.h"
#include "util/asservMath.h"


static const I2CConfig i2cconfig = {
	OPMODE_I2C,
	400000,
	STD_DUTY_CYCLE
};

constexpr uint8_t md22Address   = 0xB0;// MD22 address (All switches to ON)
constexpr uint8_t modeReg       = 0x00;
constexpr uint8_t leftMotorReg  = 0x01;
constexpr uint8_t rightMotorReg = 0x02;
constexpr uint8_t accReg        = 0x03;

constexpr uint8_t controlMode   = 0x01;// Wanted value for mode register. Ie: -128 (full reverse)   0 (stop)   127 (full forward).


Md22::Md22(bool invertMotor1, bool invertMotor2)
{
	m_invertMotor1 = invertMotor1;
	m_invertMotor2 = invertMotor2;
}

void Md22::init()
{
	// Enable I2C SDA & SCL pin
	// External pullups with correct resistance value shall be used !
	// see : http://wiki.chibios.org/dokuwiki/doku.php?id=chibios:community:guides:i2c_trouble_shooting
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

	i2cStart(&I2CD1, &i2cconfig);

	// Set mode
    uint8_t cmd[] = { modeReg, controlMode};
    msg_t msg = i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_INFINITE);

    i2c_lld_get_errors(&I2CD1);

    if (msg != MSG_OK )
    {
     // What to do ?
    }

    // Set acceleration
	cmd[0] = accReg; cmd[0] = 0;
	i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_INFINITE);

    // Set motor speed to zero
	cmd[0] = leftMotorReg; cmd[0] = 0;
	i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_INFINITE);

	cmd[0] = rightMotorReg; cmd[0] = 0;
	i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_INFINITE);
}

void Md22::setMotor1Speed(float percentage)
{
	if(m_invertMotor1)
		percentage = -percentage;

	int8_t md22SpeedConsign = (int8_t) fmap(percentage, -100.0, 100.0, -128.0, 127.0);

	uint8_t cmd[] = { leftMotorReg, (uint8_t) md22SpeedConsign};
	i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_INFINITE);
}

void Md22::setMotor2Speed(float percentage)
{
	if(m_invertMotor2)
		percentage = -percentage;

	int8_t md22SpeedConsign = (int8_t) fmap(percentage, -100.0, 100.0, -128.0, 127.0);

	uint8_t cmd[] = { rightMotorReg, (uint8_t)md22SpeedConsign};
	i2cMasterTransmitTimeout(&I2CD1, md22Address, cmd, sizeof(cmd), NULL, 0, TIME_INFINITE);
}


Md22::~Md22()
{
}

