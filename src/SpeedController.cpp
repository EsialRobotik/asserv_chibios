#include "SpeedController.h"
#include "util/asservMath.h"
#include <cmath>
#include <cstdlib>
#include <cstdint>

SpeedController::SpeedController(float speedKpSet[NB_PI_SUBSET], float speedKiSet[NB_PI_SUBSET],
        float setSpeedRange[NB_PI_SUBSET], float outputLimit, float maxInputSpeed, float measureFrequency)
{
    m_speedGoal = 0;
    m_integratedOutput = 0;
    m_speedKp = 0;
    m_speedKi = 0;

    for (int i = 0; i < NB_PI_SUBSET; i++)
    {
        m_setSpeedRange[i] = setSpeedRange[i];
        m_speedKpSet[i] = speedKpSet[i];
        m_speedKiSet[i] = speedKiSet[i];
    }

    m_outputLimit = outputLimit;
    m_inputLimit = maxInputSpeed;
    m_measureFrequency = measureFrequency;
}

float SpeedController::update(float actualSpeed)
{
    float outputValue = 0;
    float speedError = m_speedGoal - actualSpeed;

    computeGains(actualSpeed);

    // Regulateur en vitesse : un PI
    outputValue = speedError * m_speedKp;
    outputValue += m_integratedOutput;

    // On limite la sortie entre -m_outputLimit et m_outputLimit...
    bool limited = false;
    if (outputValue > m_outputLimit) {
        outputValue = m_outputLimit;
        limited = true;
    }
    if (outputValue < -m_outputLimit) {
        outputValue = -m_outputLimit;
        limited = true;
    }

    if (limited) // .. Si la sortie est limité, on désature l'intégrale
    {
        m_integratedOutput *= 0.9;
    }
    else	// .. Sinon, on integre l'erreur
    {
        m_integratedOutput += m_speedKi * speedError / m_measureFrequency;
        if (std::fabs(speedError) < 0.1) // Quand l'erreur de vitesse est proche de zero(ie: consigne à 0 et le robot ne bouge pas..), on désature l'intégrale
            m_integratedOutput *= 0.95;
    }

    // Protection antiWindup, surement inutile avec la désaturation au dessus, mais on garde ceinture & bretelles !
    if (m_integratedOutput > m_outputLimit)
        m_integratedOutput = m_outputLimit;
    else if (m_integratedOutput < -m_outputLimit)
        m_integratedOutput = -m_outputLimit;

    return outputValue;
}

void SpeedController::setSpeedGoal(float speed)
{
    if (speed > m_inputLimit)
        speed = m_inputLimit;
    if (speed < -m_inputLimit)
        speed = -m_inputLimit;

    if (speed == 0.0)
        resetIntegral();
    m_speedGoal = speed;
}
;

void SpeedController::setGains(float Kp, float Ki)
{
    m_speedKp = Kp;
    m_speedKi = Ki;
    resetIntegral();
}

void SpeedController::computeGains(float actualSpeed)
{
    // D'abord, on cherche à quel set correspond la vitesse actuelle
    uint8_t set = 0;
    while (set < NB_PI_SUBSET && actualSpeed > m_setSpeedRange[set])
        set++;

    if (set == 0) {
        // Le 1er set, on prend directement les valeurs
        m_speedKp = m_speedKpSet[0];
        m_speedKi = m_speedKiSet[0];
    } else if (set == NB_PI_SUBSET) {
        // Le dernier set, on prend directement les valeurs
        m_speedKp = m_speedKpSet[NB_PI_SUBSET - 1];
        m_speedKi = m_speedKiSet[NB_PI_SUBSET - 1];
    } else {
        // Si on se trouve entre 2 set, on fait varier linéaire les valeurs des gains
        //  en fonction de la vitesse actuelle par rapport au set courant et précedent.
        m_speedKp = fmap(actualSpeed, m_setSpeedRange[set - 1], m_setSpeedRange[set], m_speedKpSet[set - 1],
                m_speedKpSet[set]);

        m_speedKi = fmap(actualSpeed, m_setSpeedRange[set - 1], m_setSpeedRange[set], m_speedKiSet[set - 1],
                m_speedKiSet[set]);
    }
}
