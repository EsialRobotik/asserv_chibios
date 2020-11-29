#include "Goto.h"
#include "Regulator.h"
#include "util/asservMath.h"
#include "USBStream.h"
#include <new>
#include <cmath>

Goto::Goto(float consignX_mm, float consignY_mm,
        GotoConfiguration const *configuration,
        float backwardMode)
: m_consignX_mm(consignX_mm), m_consignY_mm(consignY_mm),
  m_configuration(configuration)
{
    if( backwardMode)
        m_backModeCorrection = -1;
    else
        m_backModeCorrection = 1;
}

Goto::Goto( Goto const &line)
{
    m_consignX_mm = line.m_consignX_mm;
    m_consignY_mm = line.m_consignY_mm;
    m_configuration = line.m_configuration;
    m_backModeCorrection = line.m_backModeCorrection;
}

Goto* Goto::cloneIn(Command *ptr) const
{
    return new(ptr) Goto(*this);
}

void Goto::computeInitialConsign(float X_mm, float Y_mm, float theta_rad, float *distanceConsig, float *angleConsign, const Regulator &angle_regulator, const Regulator &distance_regulator)
{
   float deltaX = m_consignX_mm - X_mm;
   float deltaY = m_consignY_mm - Y_mm;

   // Valeur absolue de la distance à parcourir en allant tout droit pour atteindre la consigne
   float deltaDist = computeDeltaDist(deltaX, deltaY);

   // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
   float deltaTheta = computeDeltaTheta(m_backModeCorrection*deltaX, m_backModeCorrection*deltaY, theta_rad);

   float projectedDist = deltaDist * cosf(deltaTheta);

   //TODO JGU: Fix pour ne pas se retourner après avoir dépassé un point sur un overshoot!
   //    mais y'a un bug, si le point est proche mais sur le coté, on va juste avancer !!!!
   if (deltaDist < m_configuration->gotoReturnThreshold_mm)
   {
       *distanceConsig = distance_regulator.getAccumulator() + m_backModeCorrection*projectedDist ;
   }
   else
   {
       *angleConsign = angle_regulator.getAccumulator() + deltaTheta;

       if (fabs(deltaTheta) < m_configuration->gotoAngleThreshold_rad)
           *distanceConsig = distance_regulator.getAccumulator() + m_backModeCorrection*deltaDist;
   }

   USBStream::instance()->setXGoal(m_consignX_mm);
   USBStream::instance()->setYGoal(m_consignY_mm);
}

void Goto::updateConsign(float X_mm, float Y_mm, float theta_rad, float *distanceConsig, float *angleConsign, const Regulator &angle_regulator, const Regulator &distance_regulator)
{
    return computeInitialConsign(X_mm, Y_mm, theta_rad, distanceConsig, angleConsign, angle_regulator, distance_regulator);
}

bool Goto::isGoalReached(float X_mm, float Y_mm, float theta_rad, const Regulator &angle_regulator, const Regulator &distance_regulator)
{
    float deltaX = m_consignX_mm - X_mm;
    float deltaY = m_consignY_mm - Y_mm;

    return computeDeltaDist(deltaX, deltaY) < m_configuration->arrivalDistanceThreshold_mm;
}

bool Goto::noStop()
{
    return false;
}

float Goto::computeDeltaDist(float deltaX, float deltaY)
{
    // On a besoin de min et max pour le calcul de la racine carrée
    float max = fabs(deltaX) > fabs(deltaY) ? fabs(deltaX) : fabs(deltaY);
    float min = fabs(deltaX) <= fabs(deltaY) ? fabs(deltaX) : fabs(deltaY);

    // Valeur absolue de la distance à parcourir en allant tout droit pour atteindre la consigne
    if (max != 0)
        return (max * sqrtf(1.0 + (min / max) * (min / max)));
    else
        return 0;
}


float Goto::computeDeltaTheta(float deltaX, float deltaY, float theta_rad)
{
    // Cap que doit atteindre le robot
    float thetaCible = atan2f(deltaY, deltaX);

    // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
    float deltaTheta = thetaCible - theta_rad;

    // On ajuste l'angle à parcourir pour ne pas faire plus d'un demi-tour
    // Exemple, tourner de 340 degrés est plus chiant que de tourner de -20 degrés
    if (deltaTheta > M_PI)
    {
        deltaTheta -= 2.0 * M_PI;
    }
    else if (deltaTheta < -M_PI)
    {
        deltaTheta += 2.0 * M_PI;
    }

    return deltaTheta;
}

