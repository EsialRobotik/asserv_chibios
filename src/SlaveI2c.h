#ifndef SRC_SLAVEI2C_H_
#define SRC_SLAVEI2C_H_

#include <ch.h>
#include <hal.h>


class SlaveI2c
{
public:

    struct I2cPinInit
    {
        stm32_gpio_t* GPIObaseSCL;
        uint8_t pinNumberSCL;
        stm32_gpio_t* GPIObaseSDA;
        uint8_t pinNumberSDA;
    };

    SlaveI2c(I2cPinInit *i2cPins, uint32_t i2cFrequency);
    ~SlaveI2c()
    {
    }


    void listen();
    //void onI2CSlaveReceive_(I2CDriver *i2cp, const uint8_t *rxbuf, size_t rxbytes);

    void init();


private:

    //typedef void (SlaveI2c::*stateFunction)(I2CDriver *i2cp, const uint8_t *rxbuf, size_t rxbytes);
    //stateFunction memberOnI2CSlaveReceive_;

    I2CConfig m_i2cconfig;
    I2cPinInit m_i2cPinConf;


    //bool startAgain_ ;
    //uint8_t *txBuff_;
    //uint8_t *rxBuff_;
};

#endif /* SRC_SLAVEI2C_H_ */
