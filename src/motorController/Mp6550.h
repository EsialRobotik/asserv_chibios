#ifndef MP6550_H_
#define MP6550_H_

#include "MotorController.h"
#include "ch.h"
#include "hal.h"
#include <string>
using namespace std;

class Mp6550: public MotorController
{
public:
    struct PwmConf_t
    {
        PWMConfig pwm_config;
        stm32_gpio_t* pwmOutpin_GPIObase;
        uint8_t pwmOutpinNumber;
        uint8_t pwmOutpin_alternate;
        uint8_t used_channel;
        PWMDriver *pwmDriver;
    };

    struct Mp6550Conf_t
    {
        PwmConf_t confPwm1Motor1;
        PwmConf_t confPwm2Motor1;
        PwmConf_t confPwm1Motor2;
        PwmConf_t confPwm2Motor2;
    };


    explicit Mp6550(Mp6550Conf_t &conf, bool is1motorRight, bool invertMotorRight, bool invertMotorLeft);
    virtual ~Mp6550() {};

    void init();
    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);


    virtual float getMotorRightSpeedNonInverted() const;
    virtual float getMotorLeftSpeedNonInverted() const;

private:
    void initPwm(PwmConf_t &pwmConf);

    Mp6550Conf_t m_mp6550Conf;
    bool m_invertMotorLeft;
    bool m_invertMotorRight;
    bool m_is1motorRight;

    int8_t m_lastRightConsign;
    int8_t m_lastLeftConsign;

    float m_rightMotorPercentage;
    float m_leftMotorPercentage;
};

#endif /* MP6550_H_ */
