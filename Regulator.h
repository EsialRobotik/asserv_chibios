/*
 * Regulator.h
 *
 *  Created on: 8 mai 2019
 *      Author: jeff
 */

#ifndef REGULATOR_H_
#define REGULATOR_H_

#include <cstdint>

class Regulator
{
public:
	explicit Regulator(float Kp);
	virtual ~Regulator(){};

	float update(float goal, float feedback);

	float getAccumulator(){ return m_accumulator;};

	void setGain(float Kp){ m_Kp = Kp;}

	void reset(){m_accumulator = 0;};

	float getError(){ return m_error;};

private:
    double m_accumulator;
    float m_Kp;
    float m_error;

};

#endif /* REGULATOR_H_ */
