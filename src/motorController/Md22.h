#ifndef MD22_H_
#define MD22_H_

#include "MotorController.h"
#include "ch.h"
#include "hal.h"
#include <string>
using namespace std;

class Md22: public MotorController
{
public:
    struct I2cPinInit
    {
        stm32_gpio_t* GPIObaseSCL;
        uint8_t pinNumberSCL;
        stm32_gpio_t* GPIObaseSDA;
        uint8_t pinNumberSDA;
    };

    explicit Md22(I2cPinInit *i2cPins, bool is1motorRight, bool invertMotorRight, bool invertMotorLeft, uint32_t i2cFrequency);
    virtual ~Md22() {};

    msg_t i2cMasterTransmitTimeoutTimes(I2CDriver *i2cp,
                                   i2caddr_t addr,
                                   const uint8_t *txbuf,
                                   size_t txbytes,
                                   uint8_t *rxbuf,
                                   size_t rxbytes,
                                   sysinterval_t timeout, int times);
    void init();
    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);

    virtual float getMotorRightSpeed() const { return m_rightMotorPercentage; };
    virtual float getMotorLeftSpeed() const { return m_leftMotorPercentage; };

    inline int8_t getRightSpeed() { return m_lastRightConsign; }
    inline int8_t getLeftSpeed() { return m_lastLeftConsign; }

private:
    I2CConfig m_i2cconfig;
    I2cPinInit m_i2cPinConf;
    bool m_invertMotorLeft;
    bool m_invertMotorRight;
    bool m_is1motorRight;

    int8_t m_lastRightConsign;
    int8_t m_lastLeftConsign;

    float m_rightMotorPercentage;
    float m_leftMotorPercentage;
};

#endif /* MD22_H_ */
