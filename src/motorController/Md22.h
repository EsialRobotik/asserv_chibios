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
        uint8_t SCLPinAlternate;
        stm32_gpio_t* GPIObaseSDA;
        uint8_t pinNumberSDA;
        uint8_t SDAPinAlternate;
    };

    explicit Md22(I2cPinInit *i2cPins, I2CDriver *i2cDriver, bool is1motorRight, bool invertMotorRight, bool invertMotorLeft, uint32_t i2cFrequency);
    virtual ~Md22() {};

    void init();
    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);


    virtual float getMotorRightSpeedNonInverted() const;
    virtual float getMotorLeftSpeedNonInverted() const;

private:
    I2cPinInit m_i2cPinConf;
    I2CDriver *m_i2cDriver;
    uint32_t m_i2cFrequency;
    bool m_invertMotorLeft;
    bool m_invertMotorRight;
    bool m_is1motorRight;

    int8_t m_lastRightConsign;
    int8_t m_lastLeftConsign;

    float m_rightMotorPercentage;
    float m_leftMotorPercentage;
};

#endif /* MD22_H_ */
