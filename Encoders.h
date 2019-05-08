/*
 * Encoders.h
 *
 *  Created on: 2 mai 2019
 *      Author: jeff
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_
#include <cstdint>

class Encoders
{
public:
	Encoders(bool invertEncoder1 = false, bool invertEncoder2 = false);
	virtual ~Encoders();

	void init();
	void start();
	void stop();


	void getValues(int16_t *encoder1, int16_t *encoder2);
	void getValuesAndReset(int16_t *encoder1, int16_t *encoder2);

private:
	bool m_invertEncoder1;
	bool m_invertEncoder2;
};

#endif /* ENCODERS_H_ */
