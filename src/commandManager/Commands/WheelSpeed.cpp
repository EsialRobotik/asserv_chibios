#include "WheelSpeed.h"
#include "Regulator.h"
#include <new>
#include <cmath>
#include <cstdio>

WheelSpeed::WheelSpeed(float rightWheelSpeed_mmpersec, float leftWheelSpeed_mmpersec, uint32_t stepDuration_ms)
 : m_rightWheelSpeed_mmpersec(rightWheelSpeed_mmpersec), m_leftWheelSpeed_mmpersec(leftWheelSpeed_mmpersec)
{
    m_step_endtime = TIME_MS2I(stepDuration_ms);
}

Command::consign_type_t WheelSpeed::computeInitialConsign(float, float , float , consign_t & consign, const Regulator &, const Regulator &)
{
    m_step_endtime += chVTGetSystemTime();
    consign.right_wheel_consign = m_rightWheelSpeed_mmpersec;
    consign.left_wheel_consign = m_leftWheelSpeed_mmpersec;

     /* /!\ 
      * This horrible hack is temporary to avoid changes in the interface 
      *  that will induce changes in the commandManager behaviour while being to close to the eurobot competition. 
      *  To be reworked during the summer 
     */
    m_consign_ref = &consign;
    return  consign_type_t::consign_wheel_speed;
}

void WheelSpeed::updateConsign(float , float , float , consign_t & , const Regulator &, const Regulator &)
{
}

bool WheelSpeed::isGoalReached(float , float , float , const Regulator &angle_regulator, const Regulator &distance_regulator, const Command*)
{
    bool isFinished = (chVTGetSystemTime() > m_step_endtime);
    if( isFinished )
    {
        // No further command, just set a consign where we are.
        m_consign_ref->type = Command::consign_type_t::consign_acceleration_limited;
        m_consign_ref->angle_consign = angle_regulator.getAccumulator();
        m_consign_ref->distance_consign = distance_regulator.getAccumulator();
    }
    return isFinished;
}

bool WheelSpeed::noStop() const
{
    return false;
}
