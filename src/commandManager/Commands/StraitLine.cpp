#include "StraitLine.h"
#include "Regulator.h"
#include <new>
#include <cmath>

StraitLine::StraitLine(float consign, float arrivalDistanceThreshold_mm)
: m_straitLineConsign(consign), m_arrivalDistanceThreshold_mm(arrivalDistanceThreshold_mm)
{
}

Command::consign_type_t StraitLine::computeInitialConsign(float , float , float , consign_t & consign, const Regulator &, const Regulator &)
{
    consign.distance_consign += m_straitLineConsign;
    return  consign_type_t::consign_acceleration_limited;
}

void StraitLine::updateConsign(float , float , float , consign_t & , const Regulator &, const Regulator &)
{
}

bool StraitLine::isGoalReached(float , float , float , const Regulator &, const Regulator &distance_regulator, const Command* )
{
    return fabs(distance_regulator.getError()) <= m_arrivalDistanceThreshold_mm;
}

bool StraitLine::noStop() const
{
    return false;
}
