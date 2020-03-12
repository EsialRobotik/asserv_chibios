#ifndef MD22_H_
#define MD22_H_

#include "MotorController.h"
#include "ch.h"
#include "hal.h"

class Md22: public MotorController
{
public:
    explicit Md22(bool is1motorRight, bool invertMotorR, bool invertMotorL);
    virtual ~Md22();

    void init();
    void setMotorLSpeed(float percentage);
    void setMotorRSpeed(float percentage);

private:
    bool m_invertMotorL;
    bool m_invertMotorR;
    bool m_is1motorRight;
};

#endif /* MD22_H_ */
