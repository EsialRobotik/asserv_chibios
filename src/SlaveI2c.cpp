#include "SlaveI2c.h"
#include <ch.h>
#include <hal.h>
#include <functional>
//
#include "hal_i2c_lld.h"


//#define CALL_MEMBER(object,ptrToMember)  ((object).*(ptrToMember))

bool startAgain = true;
uint8_t txBuff[4] = { 0x00, 0x00, 0x00, 0x00 };
uint8_t rxBuff[4] = { 0x00, 0x00, 0x00, 0x00 };

void onI2CSlaveReceive(I2CDriver *i2cp, const uint8_t *rxbuf, size_t rxbytes)
{
    (void) i2cp;
    (void) rxbuf;
    (void) rxbytes;

}

void onI2CSlaveRequest(I2CDriver *i2cp)
{

    i2cSlaveStartTransmission(i2cp, txBuff, 2);

    i2cSlaveOnReceive(&I2CD1, onI2CSlaveReceive, rxBuff, 2);

    txBuff[0]++;
    txBuff[1]++;
}

SlaveI2c::SlaveI2c(I2cPinInit *i2cPins, uint32_t i2cFrequency)
{
    m_i2cPinConf = *i2cPins;

    chDbgAssert(i2cFrequency <= 400000, "SlaveI2c: i2cFrequency shall be lower than 400khz \r\n");

    if (i2cFrequency <= 100000)
        m_i2cconfig = {OPMODE_I2C, i2cFrequency, STD_DUTY_CYCLE};
        else if (i2cFrequency > 100000)
        m_i2cconfig = {OPMODE_I2C, i2cFrequency, FAST_DUTY_CYCLE_2};

/*
    startAgain_ = true;
    rxBuff_ = new uint8_t[4];
    *rxBuff_ = {0x00};

    txBuff_ = new uint8_t[4];
    *txBuff_ = {0x00};

    memberOnI2CSlaveReceive_ = &SlaveI2c::onI2CSlaveReceive_;*/
}

void SlaveI2c::listen()
{

}
/*
void SlaveI2c::onI2CSlaveReceive_(I2CDriver *i2cp, const uint8_t *rxbuf, size_t rxbytes)
{
    (void) i2cp;
    (void) rxbuf;
    (void) rxbytes;
}*/


void SlaveI2c::init()
{
    //Start I2C Driver
    i2cStart(&I2CD3, &m_i2cconfig);

    //Set SCL and SDA pins
    palSetPadMode(m_i2cPinConf.GPIObaseSCL, m_i2cPinConf.pinNumberSCL,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(m_i2cPinConf.GPIObaseSDA, m_i2cPinConf.pinNumberSDA,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    i2cSlaveOnRequest(&I2CD3, onI2CSlaveRequest);

    i2cSlaveOnReceive(&I2CD3, onI2CSlaveReceive, rxBuff, 2);
    i2cSlaveMatchAddress(&I2CD3, 0x18);
}

/*
#define CALL_MEMBER(object,ptrToMember)  ((object).*(ptrToMember))

class  AsservStream_uartDecoder
{
private:
         typedef void (AsservStream_uartDecoder::*stateFunction)(uint8_t byte);
         stateFunction currentState;
};

void AsservStream_uartDecoder::processBytes(uint8_t *buffer, unsigned int nbBytes)
{
for(int i=0; i<nbBytes; i++)
CALL_MEMBER(*this,currentState)(buffer[i]);
}*/
