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

Command::consign_type_t OrbitalTurn::computeInitialConsign(float , float , float , consign_t & consign, const Regulator &, const Regulator &distance_regulator)
{
    consign.angle_consign += m_angleConsign;
    consign.distance_consign = distance_regulator.getAccumulator();
    return  consign_type_t::consign_polar;
}

void OrbitalTurn::updateConsign(float , float , float , consign_t & consign, const Regulator &, const Regulator &distance_regulator)
{
    consign.distance_consign = distance_regulator.getAccumulator();
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
