#include "AdaptativeSpeedController.h"
#include "util/asservMath.h"
#include "ch.h"
#include <cmath>


AdaptativeSpeedController::AdaptativeSpeedController(
        float KpGains[NB_PI_SUBSET], float KiGains[NB_PI_SUBSET], float speedRange[NB_PI_SUBSET],
        float outputLimit, float maxInputSpeed, float measureFrequency) :
        SpeedController(KpGains[0], KiGains[0], outputLimit, maxInputSpeed, measureFrequency)
{

    for (int i = 0; i < NB_PI_SUBSET; i++)
    {
        m_GainsSpeedRange[i] = speedRange[i];
        m_speedKpSet[i] = KpGains[i];
        m_speedKiSet[i] = KiGains[i];
    }
}

float AdaptativeSpeedController::update(float actualSpeed)
{
    updateGains(actualSpeed);
    return SpeedController::update(actualSpeed);
}

void AdaptativeSpeedController::updateGains(float actualSpeed)
{
    actualSpeed = fabs(actualSpeed);

    // D'abord, on cherche à quel set correspond la vitesse actuelle
    uint8_t set = 0;
    while (set < (NB_PI_SUBSET) && actualSpeed > m_GainsSpeedRange[set])
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
        m_speedKp = fmap(actualSpeed,
                m_GainsSpeedRange[set - 1], m_GainsSpeedRange[set],
                m_speedKpSet[set - 1], m_speedKpSet[set]);

        m_speedKi = fmap(actualSpeed, m_GainsSpeedRange[set - 1], m_GainsSpeedRange[set],m_speedKiSet[set - 1], m_speedKiSet[set]);

        if( m_speedKi < 0.0)
        {
            m_GainsSpeedRange[0] = 1;
            m_speedKi = fmap(actualSpeed,
                    m_GainsSpeedRange[set ], m_GainsSpeedRange[set],
                    m_speedKiSet[set ], m_speedKiSet[set]);
        }

    }
}

void AdaptativeSpeedController::setGains(float , float )
{
    chDbgAssert(false, "Call to AdaptativeSpeedController::setGains(float Kp, float Ki) has no meaning ! Call AdaptativeSpeedController::setGains(float Kp, float Ki, uint8_t range) instead");
}

void AdaptativeSpeedController::setGains(float Kp, float Ki, uint8_t range)
{
    m_speedKpSet[range] = Kp;
    m_speedKiSet[range] = Ki;
}

void AdaptativeSpeedController::getGainsForRange(uint8_t range, float *Kp, float *Ki, float *speedRange)
{
    chDbgAssert(range<NB_PI_SUBSET, "getGainsForRange with wrong range !");
    *Kp = m_speedKpSet[range];
    *Ki = m_speedKiSet[range];
    *speedRange = m_GainsSpeedRange[range];
}


Cbore & AdaptativeSpeedController::getConfiguration(Cbore & cbor_representation)
{
    return cbor_representation.map()
    .key("name").value("adv_speed_ctrl")
    .key("Range_0").array().item(m_GainsSpeedRange[0]).item(m_speedKpSet[0]).item(m_speedKiSet[0]).end()
    .key("Range_1").array().item(m_GainsSpeedRange[1]).item(m_speedKpSet[1]).item(m_speedKiSet[1]).end()
    .key("Range_2").array().item(m_GainsSpeedRange[2]).item(m_speedKpSet[2]).item(m_speedKiSet[2]).end()
    .end();
}

