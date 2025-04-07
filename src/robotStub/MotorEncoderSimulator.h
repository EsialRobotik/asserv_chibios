#ifndef MOTOR_CONTROLLER_SIMULATION_H_
#define MOTOR_CONTROLLER_SIMULATION_H_

#include "MotorController.h"
#include "Encoder.h"
#include "ch.h"
#include "FirstOrderSim.h"
#include <string>

using namespace std;


class Odometry;


class MotorEncoderSimulator: public MotorController, public Encoders
{
public:

    explicit MotorEncoderSimulator(float period, float wheelRadius_mm, float encodersTicksByTurn, Odometry *odometry);
    virtual ~MotorEncoderSimulator() {};

    void setMotorRightSpeed(float percentage);
    void setMotorLeftSpeed(float percentage);


    virtual float getMotorRightSpeedNonInverted() const;
    virtual float getMotorLeftSpeedNonInverted() const;


    virtual void getValues(float *deltaEncoderRight, float *deltaEncoderLeft);

    void backwarkPertubation();

private:

    float m_rightMotorPercentage;
    float m_leftMotorPercentage;

    float m_encoderR;
    float m_encoderL;

    FirstOrderSim m_motorRightSim;
    FirstOrderSim m_motorLeftSim;

    float m_encodermmByTicks;

    Odometry *m_odometry;

    bool m_backwarkPerturbation;
    systime_t m_backwardPertubation_start_time;
};

#endif /* MOTOR_CONTROLLER_SIMULATION_H_ */

