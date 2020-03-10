#ifndef ENCODERS_H_
#define ENCODERS_H_

#include "Encoder.h"

class QuadratureEncoder : public Encoders
{
public:
	QuadratureEncoder(bool invertEncoder1 = false, bool invertEncoder2 = false);
	virtual ~QuadratureEncoder();

	void init();
	void start();
	void stop();

	void getEncodersTotalCount(int64_t * encoderRight, int64_t * encoderLeft);

	virtual void getValues(int16_t *encoderRight, int16_t *encoderLeft);

private:
	bool m_invertEncoder1;
	bool m_invertEncoder2;
	int64_t m_encoder1Sum;
	int64_t m_encoder2Sum;
	int16_t m_encoder1Previous;
    int16_t m_encoder2Previous;
};

#endif /* ENCODERS_H_ */
