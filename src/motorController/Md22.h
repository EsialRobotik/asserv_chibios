#ifndef MD22_H_
#define MD22_H_

#include "MotorController.h"
#include "ch.h"
#include "hal.h"

// Définition des adresses
#define MD22_ADDRESS          0xB0>>1    //Adresse de la MD22 ( pour tous les switchs de mode à ON)
#define MD22_MODEREG          0x00       //Registre de mode
#define MD22_MOTOR2REG        0x02     //Registre vitesse moteur gauche ) correspondent aux numéros
#define MD22_MOTOR1REG        0x01     //Registre vitesse moteur droit  ) des moteurs sur la MD22
#define MD22_ACCREG           0x03         //Registre d'accélération

// register value configuration
#define MD22_CONTROLMODE      0x01    //Mode (0x01 : vitesse allant de -128 to 127)
#define MD22_ACC              0x00        //Accélération

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
    I2CConfig m_i2cconfig;
    I2cPinInit m_i2cPinConf;
    bool m_invertMotorLeft;
    bool m_invertMotorRight;
    bool m_is1motorRight;
    int m_freq;
};

#endif /* MD22_H_ */
