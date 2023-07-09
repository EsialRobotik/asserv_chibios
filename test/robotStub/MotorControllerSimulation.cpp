#include <ch.h>
#include <hal.h>
#include "MotorControllerSimulation.h"
#include "util/asservMath.h"


 MotorControllerSimulation::MotorControllerSimulation() :
        MotorController()
{
    m_rightMotorPercentage = 0;
    m_leftMotorPercentage = 0;

}


void MotorControllerSimulation::setMotorLeftSpeed(float percentage)
{
    m_leftMotorPercentage = limit(percentage, -100.0, 100.0);
}

void MotorControllerSimulation::setMotorRightSpeed(float percentage)
{
    m_rightMotorPercentage = limit(percentage, -100.0, 100.0);
}

float MotorControllerSimulation::getMotorRightSpeedNonInverted() const
{
        return m_rightMotorPercentage;
}

float MotorControllerSimulation::getMotorLeftSpeedNonInverted() const
{
        return m_leftMotorPercentage;
}

