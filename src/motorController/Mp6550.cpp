#include "Mp6550.h"
#include <ch.h>
#include <hal.h>
#include "util/asservMath.h"
#include "motorController/Mp6550.h"
#include <chprintf.h>


extern BaseSequentialStream *outputStream;

 Mp6550::Mp6550(Mp6550Conf_t &conf, bool is1motorRight, bool invertMotorRight, bool invertMotorLeft) :
        MotorController()
{
    m_mp6550Conf = conf;
    m_invertMotorRight = invertMotorRight;
    m_invertMotorLeft = invertMotorLeft;
    m_is1motorRight = is1motorRight;
    m_lastRightConsign = 0;
    m_lastLeftConsign = 0;
    m_rightMotorPercentage = 0;
    m_leftMotorPercentage = 0;

}

void Mp6550::initPwm(PwmConf_t &pwmConf)
{
    pwmStart(pwmConf.pwmDriver, &pwmConf.pwm_config);
    pwmEnableChannel(pwmConf.pwmDriver, pwmConf.used_channel, PWM_PERCENTAGE_TO_WIDTH(pwmConf.pwmDriver, 0));
    palSetPadMode(pwmConf.pwmOutpin_GPIObase, pwmConf.pwmOutpinNumber, PAL_MODE_ALTERNATE(pwmConf.pwmOutpin_alternate));
}

void Mp6550::init()
{
    initPwm(m_mp6550Conf.confPwm1Motor1);
    initPwm(m_mp6550Conf.confPwm2Motor1);
    initPwm(m_mp6550Conf.confPwm1Motor2);
    initPwm(m_mp6550Conf.confPwm2Motor2);
}

void Mp6550::setMotorLeftSpeed(float percentage)
{
    m_leftMotorPercentage = limit(percentage, -100.0, 100.0);

    if (m_invertMotorLeft)
        m_leftMotorPercentage = -m_leftMotorPercentage;

    PwmConf_t *pwm1;
    PwmConf_t *pwm2;
    if (m_is1motorRight)
    {
        pwm1 = &m_mp6550Conf.confPwm1Motor2;
        pwm2 = &m_mp6550Conf.confPwm2Motor2;
    }
    else
    {
        pwm1 = &m_mp6550Conf.confPwm1Motor1;
        pwm2 = &m_mp6550Conf.confPwm2Motor1;
    }

    uint32_t duty_cycle = 0;

    if( m_leftMotorPercentage > 0.0 )
    {
        // In brake mode, consign are mapped as : (100%-pwm%) , See https://www.pololu.com/product/4733
        // Map between 0 and 10'000 to comply with PWM_PERCENTAGE_TO_WIDTH()
        duty_cycle = limit((100.-m_leftMotorPercentage)*100, 0, 10000);
        pwmEnableChannel(pwm1->pwmDriver, pwm1->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm1->pwmDriver, duty_cycle));
        pwmEnableChannel(pwm2->pwmDriver, pwm2->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm2->pwmDriver, 10000));
    }
    else if( m_leftMotorPercentage < 0.0)
    {
        // In brake mode, consign are mapped as : (100%-pwm%) , See https://www.pololu.com/product/4733
        // Map between 0 and 10'000 to comply with PWM_PERCENTAGE_TO_WIDTH()
        duty_cycle = limit((100.-(-1.*m_leftMotorPercentage))*100, 0, 10000);
        pwmEnableChannel(pwm1->pwmDriver, pwm1->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm1->pwmDriver, 10000));
        pwmEnableChannel(pwm2->pwmDriver, pwm2->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm2->pwmDriver, duty_cycle));
    }
    else
    {
        // Set both pwm to 1 to brake
       pwmEnableChannel(pwm1->pwmDriver, pwm1->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm1->pwmDriver, 10000));
       pwmEnableChannel(pwm2->pwmDriver, pwm2->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm2->pwmDriver, 10000));
    }

    chprintf(outputStream, "setMotorLeftSpeed %f -> %f : duty_cycle %d \r\n", percentage, m_leftMotorPercentage, duty_cycle);

}

void Mp6550::setMotorRightSpeed(float percentage)
{
    m_rightMotorPercentage = limit(percentage, -100.0, 100.0);

    if (m_invertMotorRight)
        m_rightMotorPercentage = -m_rightMotorPercentage;

    PwmConf_t *pwm1;
    PwmConf_t *pwm2;
    if (m_is1motorRight)
    {
       pwm1 = &m_mp6550Conf.confPwm1Motor1;
       pwm2 = &m_mp6550Conf.confPwm2Motor1;
    }
    else
    {
       pwm1 = &m_mp6550Conf.confPwm1Motor2;
       pwm2 = &m_mp6550Conf.confPwm2Motor2;
    }

    uint32_t duty_cycle = 0;

    if( m_rightMotorPercentage > 0.0 )
    {
       // In brake mode, consign are mapped as : (100%-pwm%) , See https://www.pololu.com/product/4733
       // Map between 0 and 10'000 to comply with PWM_PERCENTAGE_TO_WIDTH()
       duty_cycle = limit((100.-m_rightMotorPercentage)*100, 0, 10000);
       pwmEnableChannel(pwm1->pwmDriver, pwm1->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm1->pwmDriver, duty_cycle));
       pwmEnableChannel(pwm2->pwmDriver, pwm2->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm2->pwmDriver, 10000));
    }
    else if( m_rightMotorPercentage < 0.0)
    {
       // In brake mode, consign are mapped as : (100%-pwm%) , See https://www.pololu.com/product/4733
       // Map between 0 and 10'000 to comply with PWM_PERCENTAGE_TO_WIDTH()
       duty_cycle = limit((100.-(-1.*m_rightMotorPercentage))*100, 0, 10000);
       pwmEnableChannel(pwm1->pwmDriver, pwm1->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm1->pwmDriver, 10000));
       pwmEnableChannel(pwm2->pwmDriver, pwm2->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm2->pwmDriver, duty_cycle));
    }
    else
    {
       // Set both pwm to 1 to brake
      pwmEnableChannel(pwm1->pwmDriver, pwm1->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm1->pwmDriver, 10000));
      pwmEnableChannel(pwm2->pwmDriver, pwm2->used_channel, PWM_PERCENTAGE_TO_WIDTH(pwm2->pwmDriver, 10000));
    }

    chprintf(outputStream, "setMotorRightSpeed %f -> %f : duty_cycle %d \r\n", percentage, m_rightMotorPercentage, duty_cycle);
}

float Mp6550::getMotorRightSpeedNonInverted() const
{
    if (m_invertMotorRight)
        return -m_rightMotorPercentage;
    else
        return m_rightMotorPercentage;
}

float Mp6550::getMotorLeftSpeedNonInverted() const
{
    if (m_invertMotorLeft)
        return -m_leftMotorPercentage;
    else
        return m_leftMotorPercentage;
}

