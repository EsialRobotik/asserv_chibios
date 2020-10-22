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

    explicit Md22(bool is1motorRight, bool invertMotorRight, bool invertMotorLeft, I2cPinInit *i2cPins, uint32_t i2cFrequency);
    virtual ~Md22() {};

    void init();
    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);

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
};

#endif /* MD22_H_ */
