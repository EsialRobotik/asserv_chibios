#include "Pll.h"
#include <cmath>

Pll::Pll(float bandwidth)
{
	setBandwidth(bandwidth);
	m_position = 0;
	m_speed = 0;
	m_count = 0;
}

void Pll::update(int16_t deltaTicks, float deltaT)
{
	// Prediction
    m_position += deltaT * m_speed;

    // Calul de l'erreur entre la prediction et l'info codeur
    m_count += deltaTicks;
    float deltaPos = (float)(m_count - (int64_t)floor(m_position));

	// Correction de la PLL
	m_position += deltaT * m_kp * deltaPos;
	m_speed += deltaT * m_ki * deltaPos;

    if (fabsf(m_speed) < 0.5f * deltaT * m_ki)
        m_speed = 0.0f;
}


void Pll::setBandwidth(float bandwidth)
{
	m_kp = 2.0f * bandwidth;
	m_ki = 0.25f * m_kp * m_kp;
}
