#ifndef MD22_H_
#define MD22_H_

#include "MotorController.h"
#include "ch.h"
#include "hal.h"

class Md22: public MotorController
{
public:
    explicit Md22(bool is1motorRight, bool invertMotorRight, bool invertMotorLeft);
    virtual ~Md22() {};

    typedef struct I2cPinInit
    {
        stm32_gpio_t* GPIObaseSCL;
        uint8_t pinNumberSCL;
        stm32_gpio_t* GPIObaseSDA;
        uint8_t pinNumberSDA;
    } I2cPinInit_t;

    static I2cPinInit_t esialCardPinConf;
    static I2cPinInit_t PMXCardPinConf;

    void init(I2cPinInit_t *I2cPinInitConf);
    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);

private:
    bool m_invertMotorLeft;
    bool m_invertMotorRight;
    bool m_is1motorRight;
};

#endif /* MD22_H_ */
