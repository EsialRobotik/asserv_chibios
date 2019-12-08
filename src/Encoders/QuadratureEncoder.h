#ifndef ENCODERS_H_
#define ENCODERS_H_

#include "Encoder.h"

class QuadratureEncoder : public Encoders
{
public:
	QuadratureEncoder(bool invertEncoder1 = false, bool invertEncoder2 = false, float encoder1Ratio=1, float encoder2Ratio=1);
	virtual ~QuadratureEncoder();

	virtual void init();
	virtual void start();
	virtual void stop();

	virtual void getValuesAndReset(int16_t *encoder1, int16_t *encoder2);

private:
	bool m_invertEncoder1;
	bool m_invertEncoder2;
	float m_encoder1Ratio;
	float m_encoder2Ratio;
};

#endif /* ENCODERS_H_ */
