#include "WheelSpeed.h"
#include "Regulator.h"
#include <new>
#include <cmath>

WheelSpeed::WheelSpeed(float rightWheelSpeed_mmpersec, float leftWheelSpeed_mmpersec, uint32_t stepDuration_ms)
 : m_rightWheelSpeed_mmpersec(rightWheelSpeed_mmpersec), m_leftWheelSpeed_mmpersec(leftWheelSpeed_mmpersec)
{
    m_step_endtime = chVTGetSystemTime();
    m_step_endtime += TIME_US2I(stepDuration_ms);
}

void WheelSpeed::computeInitialConsign(float, float , float , float *distanceConsign, float *angleConsign, const Regulator &, const Regulator &)
{
    *angleConsign = m_rightWheelSpeed_mmpersec;
    *distanceConsign = m_leftWheelSpeed_mmpersec;
}

void WheelSpeed::updateConsign(float , float , float , float *, float *, const Regulator &, const Regulator &)
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
