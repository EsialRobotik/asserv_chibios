#ifndef MD22_H_
#define MD22_H_

#include "MotorController.h"
#include "ch.h"
#include "hal.h"

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

    explicit Md22(bool is1motorRight, bool invertMotorRight, bool invertMotorLeft, I2cPinInit pins, int freq=100);
    virtual ~Md22() {};

    void init();
    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);

private:
    I2cPinInit m_i2cPinConf;
    bool m_invertMotorLeft;
    bool m_invertMotorRight;
    bool m_is1motorRight;
    int m_freq;
};

#endif /* MD22_H_ */
