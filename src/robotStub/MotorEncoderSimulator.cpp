#include <ch.h>
#include <hal.h>
#include "MotorEncoderSimulator.h"
#include "util/asservMath.h"
#include <cmath>
#include "Odometry.h"


#define MAX_SPEED 3000
#define TAU 0.2

 MotorEncoderSimulator::MotorEncoderSimulator(float period, float wheelRadius_mm, float encodersTicksByTurn, Odometry *odometry) :
        MotorController(), Encoders(),
        m_motorRightSim(period, TAU, MAX_SPEED/100), m_motorLeftSim(period, TAU, MAX_SPEED/100),
		m_odometry(odometry), m_backwarkPerturbation(false),m_backwardPertubation_start_time(0)
{
    m_rightMotorPercentage = 0;
    m_leftMotorPercentage = 0;
    m_encoderR = 0;
    m_encoderL = 0;

    float m_distanceByEncoderTurn_mm(M_2PI * wheelRadius_mm);
    m_encodermmByTicks = encodersTicksByTurn / m_distanceByEncoderTurn_mm ;
}


void MotorEncoderSimulator::setMotorLeftSpeed(float percentage)
{
    m_leftMotorPercentage = limit(percentage, -100.0, 100.0);
}

void MotorEncoderSimulator::setMotorRightSpeed(float percentage)
{
    m_rightMotorPercentage = limit(percentage, -100.0, 100.0);
}

float MotorEncoderSimulator::getMotorRightSpeedNonInverted() const
{
    return m_rightMotorPercentage;
}

float MotorEncoderSimulator::getMotorLeftSpeedNonInverted() const
{
    return m_leftMotorPercentage;
}


void MotorEncoderSimulator::getValues(float *deltaEncoderRight, float *deltaEncoderLeft)
{
    if( fabs(m_rightMotorPercentage) < 5 )
        m_rightMotorPercentage = 0;

    if( fabs(m_leftMotorPercentage) < 5 )
        m_leftMotorPercentage = 0;




    if( m_backwarkPerturbation )
    {
//        if( (chVTGetSystemTime() - m_backwardPertubation_start_time) > 500)
//        {
//            *deltaEncoderRight = 0;
//            *deltaEncoderLeft = 0;
//        }
//        else
        {
            float rightWheelSpeed_mm = m_motorRightSim.process(-1);
            float leftWheelSpeed_mm = m_motorLeftSim.process(-1);
            *deltaEncoderRight = (rightWheelSpeed_mm/10.0);
            *deltaEncoderLeft = (leftWheelSpeed_mm /10.0);
        }


        if( (chVTGetSystemTime() - m_backwardPertubation_start_time) > 1000)
        {
            m_backwarkPerturbation=false;
        }
    }
    else
    {
        float rightWheelSpeed_mm = m_motorRightSim.process(m_rightMotorPercentage);
        float leftWheelSpeed_mm = m_motorLeftSim.process(m_leftMotorPercentage);
        *deltaEncoderRight = (rightWheelSpeed_mm/10);
        *deltaEncoderLeft = (leftWheelSpeed_mm /10);
    }
}


void MotorEncoderSimulator::backwarkPertubation()
{
    m_backwarkPerturbation = true;
    m_backwardPertubation_start_time = chVTGetSystemTime();
}

