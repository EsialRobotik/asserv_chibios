#include "OrbitalTurn.h"
#include "Regulator.h"
#include <new>
#include <cmath>

OrbitalTurn::OrbitalTurn(float consign_rad, bool forward, bool turnToTheRight, float arrivalAngleThreshold_rad)
 : m_arrivalAngleThreshold_rad(arrivalAngleThreshold_rad)
{
    m_angleConsign = consign_rad;

    if( turnToTheRight )
    {
        m_mixing_type = AsservMain::mixing_type_angle_regulator_left_wheel_inverted_only;
        if( forward )
            m_angleConsign = -m_angleConsign;
    }
    else
    {
        m_mixing_type = AsservMain::mixing_type_angle_regulator_right_wheel_only;
        if( !forward)
            m_angleConsign = -m_angleConsign;
    }
}

void OrbitalTurn::computeInitialConsign(float , float , float , float *distanceConsig, float *angleConsign, const Regulator &, const Regulator &distance_regulator)
{
    *angleConsign += m_angleConsign;
    *distanceConsig = distance_regulator.getAccumulator();
}

void OrbitalTurn::updateConsign(float , float , float , float *distanceConsig, float *, const Regulator &, const Regulator &distance_regulator)
{
    *distanceConsig = distance_regulator.getAccumulator();
}

bool OrbitalTurn::isGoalReached(float , float , float , const Regulator &angle_regulator, const Regulator &, const Command* )
{
    return fabs(angle_regulator.getError()) <= m_arrivalAngleThreshold_rad;
}

bool OrbitalTurn::noStop() const
{
    return false;
}


AsservMain::mixing_type_t OrbitalTurn::getMixingType() const
{
    return m_mixing_type;
}
