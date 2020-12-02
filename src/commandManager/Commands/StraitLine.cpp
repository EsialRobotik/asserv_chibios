#include "StraitLine.h"
#include "Regulator.h"
#include <new>
#include <cmath>

StraitLine::StraitLine(float consign, float arrivalDistanceThreshold_mm)
: m_straitLineConsign(consign), m_arrivalDistanceThreshold_mm(arrivalDistanceThreshold_mm)
{
}

void StraitLine::computeInitialConsign(float , float , float , float *distanceConsig, float *, const Regulator &, const Regulator &)
{
    *distanceConsig += m_straitLineConsign;
}

void StraitLine::updateConsign(float , float , float , float *, float *, const Regulator &, const Regulator &)
{
}

bool StraitLine::isGoalReached(float , float , float , const Regulator &, const Regulator &distance_regulator)
{
    return fabs(distance_regulator.getError()) <= m_arrivalDistanceThreshold_mm;
}

bool StraitLine::noStop()
{
    return false;
}
