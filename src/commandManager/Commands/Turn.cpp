#include "Turn.h"
#include "Regulator.h"
#include <new>
#include <cmath>

Turn::Turn(float consign_rad, float arrivalAngleThreshold_rad)
 : m_angleConsign(consign_rad), m_arrivalAngleThreshold_rad(arrivalAngleThreshold_rad)
{
}

Command::consign_type_t Turn::computeInitialConsign(float , float , float , consign_t & consign, const Regulator &, const Regulator &)
{
    consign.angle_consign += m_angleConsign;
    return  consign_type_t::consign_acceleration_limited;
}

void Turn::updateConsign(float , float , float , consign_t & , const Regulator &, const Regulator &)
{
}

bool Turn::isGoalReached(float , float , float , const Regulator &angle_regulator, const Regulator &, const Command* )
{
    return fabs(angle_regulator.getError()) <= m_arrivalAngleThreshold_rad;
}

bool Turn::noStop() const
{
    return false;
}
