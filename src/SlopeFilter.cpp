#include "SlopeFilter.h"

SlopeFilter::SlopeFilter(float maxSlope)
{
	m_maxSlope = maxSlope;
	m_lastOutput = 0;
}

float SlopeFilter::filter(float dt, float value)
{
    float change = value - m_lastOutput;
    float maxDelta = dt * m_maxSlope;
    m_lastOutput += constrain(change, -maxDelta, maxDelta);
    return m_lastOutput;
}

void SlopeFilter::setSlope(float slope)
{
	m_lastOutput = 0;
	m_maxSlope = slope;
}

float SlopeFilter::constrain(float value, float low, float high) {
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}
