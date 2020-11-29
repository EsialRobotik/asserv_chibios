#include "GotoNoStop.h"
#include "Regulator.h"
#include "util/asservMath.h"
#include <new>
#include <cmath>

GotoNoStop::GotoNoStop(float consignX_mm, float consignY_mm,
        GotoConfiguration const *configuration)
: m_consignX_mm(consignX_mm), m_consignY_mm(consignY_mm),
  m_configuration(configuration)
{
}

GotoNoStop::GotoNoStop( GotoNoStop const &line)
{
    m_consignX_mm = line.m_consignX_mm;
    m_consignY_mm = line.m_consignY_mm;
    m_configuration = line.m_configuration;
}

GotoNoStop* GotoNoStop::cloneIn(Command *ptr) const
{
    return new(ptr) GotoNoStop(*this);
}

void GotoNoStop::computeInitialConsign(float X_mm, float Y_mm, float theta_rad, float *distanceConsig, float *angleConsign, const Regulator &angle_regulator, const Regulator &distance_regulator)
{
   float deltaX = m_consignX_mm - X_mm;
   float deltaY = m_consignY_mm - Y_mm;

   // Valeur absolue de la distance à parcourir en allant tout droit pour atteindre la consigne
   float deltaDist = computeDeltaDist(deltaX, deltaY);

   // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
   float deltaTheta = computeDeltaTheta(m_backModeCorrection*deltaX, m_backModeCorrection*deltaY, theta_rad);

   float projectedDist = deltaDist * cos(deltaTheta);

   //TODO JGU: Fix pour ne pas se retourner après avoir dépassé un point sur un overshoot!
   //    mais y'a un bug, si le point est proche mais sur le coté, on va juste avancer !!!!
   if (deltaDist < m_configuration->gotoReturnThreshold_mm)
   {
       *distanceConsig += m_backModeCorrection*projectedDist ;
   }
   else
   {
       *angleConsign += deltaTheta;

       if (fabs(deltaTheta) < m_configuration->gotoAngleThreshold_rad)
           *distanceConsig += m_backModeCorrection*deltaDist;
   }
}

void GotoNoStop::updateConsign(float X_mm, float Y_mm, float theta_rad, float *distanceConsig, float *angleConsign, const Regulator &angle_regulator, const Regulator &distance_regulator)
{
    return computeInitialConsign(X_mm, Y_mm, theta_rad, distanceConsig, angleConsign, angle_regulator, distance_regulator);
}

bool GotoNoStop::isGoalReached(float , float , float , const Regulator &angle_regulator, const Regulator &distance_regulator)
{
    return (fabs(distance_regulator.getError()) <= m_configuration->arrivalDistanceThreshold_mm)
            && (fabs(angle_regulator.getError()) <= m_configuration->arrivalAngleThreshold_rad);
}

bool GotoNoStop::noStop()
{
    return false;
}
