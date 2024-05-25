#include "WheelSpeed.h"
#include "Regulator.h"
#include <new>
#include <cmath>
#include <cstdio>

WheelSpeed::WheelSpeed(float rightWheelSpeed_mmpersec, float leftWheelSpeed_mmpersec, uint32_t stepDuration_ms)
 : m_rightWheelSpeed_mmpersec(rightWheelSpeed_mmpersec), m_leftWheelSpeed_mmpersec(leftWheelSpeed_mmpersec)
{
    m_step_endtime = chVTGetSystemTime();
    m_step_endtime += TIME_US2I(stepDuration_ms);
}

Command::consign_type_t WheelSpeed::computeInitialConsign(float, float , float , consign_t & consign, const Regulator &, const Regulator &)
{
    consign.right_wheel_consign = m_rightWheelSpeed_mmpersec;
    consign.left_wheel_consign = m_leftWheelSpeed_mmpersec;
    return  consign_type_t::consign_wheel_speed;
}

void WheelSpeed::updateConsign(float , float , float , consign_t & , const Regulator &, const Regulator &)
{
}

bool WheelSpeed::isGoalReached(float , float , float , const Regulator &, const Regulator &, const Command* )
{
    return chVTGetSystemTime() > m_step_endtime;
}

bool WheelSpeed::noStop() const
{
    return false;
}
