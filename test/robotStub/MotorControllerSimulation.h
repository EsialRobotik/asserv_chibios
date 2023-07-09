#ifndef MOTOR_CONTROLLER_SIMULATION_H_
#define MOTOR_CONTROLLER_SIMULATION_H_

#include "MotorController.h"
#include <string>
using namespace std;

class MotorControllerSimulation: public MotorController
{
public:

    explicit MotorControllerSimulation();
    virtual ~MotorControllerSimulation() {};

    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);


    virtual float getMotorRightSpeed() const;
    virtual float getMotorLeftSpeed() const;

private:

    float m_rightMotorPercentage;
    float m_leftMotorPercentage;
};

#endif /* MOTOR_CONTROLLER_SIMULATION_H_ */
