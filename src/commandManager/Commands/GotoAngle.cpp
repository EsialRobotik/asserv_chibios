#include "GotoAngle.h"
#include "Regulator.h"
#include "Goto.h"
#include "util/asservMath.h"
#include <new>
#include <cmath>

GotoAngle::GotoAngle(float consignX_mm, float consignY_mm,
        float arrivalAngleThreshold_rad)
: m_consignX_mm(consignX_mm), m_consignY_mm(consignY_mm),
  m_arrivalAngleThreshold_rad(arrivalAngleThreshold_rad)
{
}

void GotoAngle::computeInitialConsign(float X_mm, float Y_mm, float theta_rad, float *, float *angleConsign, const Regulator &angle_regulator, const Regulator &)
{
   float deltaX = m_consignX_mm - X_mm;
   float deltaY = m_consignY_mm - Y_mm;

   // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
   float deltaTheta = Goto::computeDeltaTheta(deltaX, deltaY, theta_rad);

   *angleConsign =  angle_regulator.getAccumulator() + deltaTheta;
}

void GotoAngle::updateConsign(float X_mm, float Y_mm, float theta_rad, float *distanceConsig, float *angleConsign, const Regulator &angle_regulator, const Regulator &distance_regulator)
{
    return computeInitialConsign(X_mm, Y_mm, theta_rad, distanceConsig, angleConsign, angle_regulator, distance_regulator);
}


bool GotoAngle::isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &, const Regulator &)
{
    float deltaX = m_consignX_mm - X_mm;
    float deltaY = m_consignY_mm - Y_mm;

    return fabs(Goto::computeDeltaTheta(deltaX, deltaY, theta_rad)) < m_arrivalAngleThreshold_rad;
}

bool GotoAngle::noStop()
{
    return false;
}
