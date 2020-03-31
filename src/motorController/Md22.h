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

    void init();
    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);

private:
    bool m_invertMotorLeft;
    bool m_invertMotorRight;
    bool m_is1motorRight;
};

#endif /* MD22_H_ */
