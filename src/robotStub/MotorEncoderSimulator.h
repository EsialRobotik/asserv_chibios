#ifndef MOTOR_CONTROLLER_SIMULATION_H_
#define MOTOR_CONTROLLER_SIMULATION_H_

#include "motorController/MotorController.h"
#include "Encoders/Encoder.h"
#include "FirstOrderSim.h"
#include <string>

using namespace std;


class MotorEncoderSimulator: public MotorController, public Encoders
{
public:

    explicit MotorEncoderSimulator(float period, float wheelRadius_mm, float encodersTicksByTurn);
    virtual ~MotorEncoderSimulator() {};

    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);


    virtual float getMotorRightSpeedNonInverted() const;
    virtual float getMotorLeftSpeedNonInverted() const;


    virtual void getValues(float *deltaEncoderRight, float *deltaEncoderLeft);

private:

    float m_rightMotorPercentage;
    float m_leftMotorPercentage;

    float m_encoderR;
    float m_encoderL;

    FirstOrderSim m_motorRightSim;
    FirstOrderSim m_motorLeftSim;

    float m_encodermmByTicks;
};

#endif /* MOTOR_CONTROLLER_SIMULATION_H_ */

