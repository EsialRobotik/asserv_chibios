/*
 * Regulator.cpp
 *
 *  Created on: 8 mai 2019
 *      Author: jeff
 */

#include "Regulator.h"

Regulator::Regulator(float Kp)
{
	m_accumulator = 0;
	m_Kp = Kp;
}

float Regulator::update(float goal, float feedback)
{
	m_accumulator += feedback;
	float error = goal-m_accumulator;
	return error*m_Kp;
}

