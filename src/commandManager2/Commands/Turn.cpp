#include "Turn.h"
#include "Regulator.h"
#include <new>
#include <cmath>

Turn::Turn(float consign_rad, float arrivalAngleThreshold_rad)
 : m_angleConsign(consign_rad), m_arrivalAngleThreshold_rad(arrivalAngleThreshold_rad)
{
}

Turn::Turn( Turn const &line)
{
    m_angleConsign = line.m_angleConsign;
    m_arrivalAngleThreshold_rad = line.m_arrivalAngleThreshold_rad;
}

Turn* Turn::cloneIn(Command *ptr) const
{
    return new(ptr) Turn(*this);
}

void Turn::computeInitialConsign(float , float , float , float *, float *angleConsign, const Regulator &, const Regulator &)
{
    *angleConsign += m_angleConsign;
}

void Turn::updateConsign(float , float , float , float *, float *, const Regulator &, const Regulator &)
{
}

bool Turn::isGoalReached(float , float , float , const Regulator &angle_regulator, const Regulator &)
{
    return fabs(angle_regulator.getError()) <= m_arrivalAngleThreshold_rad;
}

bool Turn::noStop()
{
    return false;
}
