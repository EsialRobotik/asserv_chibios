#ifndef ENCODERS_H_
#define ENCODERS_H_

#include "Encoder.h"

class QuadratureEncoder: public Encoders
{
public:
    QuadratureEncoder(bool is1EncoderRight, bool invertEncoderR = false, bool invertEncoderL = false);
    virtual ~QuadratureEncoder();

    void init();
    void start();
    void stop();

    void getEncodersTotalCount(int32_t *encoderRight, int32_t *encoderLeft);

    virtual void getValues(int16_t *deltaEncoderRight, int16_t *deltaEncoderLeft);

private:
    bool m_invertEncoderL;
    bool m_invertEncoderR;
    int32_t m_encoderLSum;
    int32_t m_encoderRSum;
    int16_t m_encoder1Previous;
    int16_t m_encoder2Previous;
    bool m_is1EncoderRight;
};

#endif /* ENCODERS_H_ */
