#ifndef MD22_H_
#define MD22_H_

#include "MotorController.h"
#include "ch.h"
#include "hal.h"

class Md22: public MotorController
{
public:
    explicit Md22(bool invertMotor1, bool invertMotor2);
    virtual ~Md22();

    void init();
    void setMotor1Speed(float percentage);
    void setMotor2Speed(float percentage);

private:
    bool m_invertMotor1;
    bool m_invertMotor2;
};

#endif /* MD22_H_ */
