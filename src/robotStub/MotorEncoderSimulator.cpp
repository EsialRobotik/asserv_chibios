#include <ch.h>
#include <hal.h>
#include "MotorEncoderSimulator.h"
#include "util/asservMath.h"
#include <cmath>
#include "Odometry.h"


#define MAX_SPEED 3000
#define TAU 0.4

 MotorEncoderSimulator::MotorEncoderSimulator(float period, float wheelRadius_mm, float encodersTicksByTurn, Odometry *odometry) :
        MotorController(), Encoders(),
        m_motorRightSim(period, TAU, MAX_SPEED/100), m_motorLeftSim(period, TAU, MAX_SPEED/100),
		m_odometry(odometry)
{
    m_rightMotorPercentage = 0;
    m_leftMotorPercentage = 0;
    m_encoderR = 0;
    m_encoderL = 0;

    float m_distanceByEncoderTurn_mm(M_2PI * wheelRadius_mm);
    m_encodermmByTicks = encodersTicksByTurn / m_distanceByEncoderTurn_mm ;
    printf("m_encodermmByTicks %f \n", m_encodermmByTicks);
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


    float rightWheelSpeed_mm = m_motorRightSim.process(m_rightMotorPercentage);
    float leftWheelSpeed_mm = m_motorLeftSim.process(m_leftMotorPercentage);

    if( m_odometry->getX() < -10 )
    {
    	m_motorRightSim.reset();
    	m_motorLeftSim.reset();
    	rightWheelSpeed_mm = 0;
    	leftWheelSpeed_mm = 0;
    }


    *deltaEncoderRight = (rightWheelSpeed_mm/10);
    *deltaEncoderLeft = (leftWheelSpeed_mm /10);
}

