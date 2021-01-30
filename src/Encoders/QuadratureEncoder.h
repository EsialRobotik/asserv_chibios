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

    int32_t getRightEncoderTotalCount() { return m_encoderRSum;};
    int32_t getLeftEncoderTotalCount() {return m_encoderLSum;};

    float getRightEncoderGain() { return m_encoderRGain; };
    float getLeftEncoderGain() { return m_encoderLGain; };

    virtual void getValues(float *deltaEncoderRight, float *deltaEncoderLeft);

private:
    bool m_invertEncoderL;
    bool m_invertEncoderR;
    int32_t m_encoderLSum;
    int32_t m_encoderRSum;
    int16_t m_encoder1Previous;
    int16_t m_encoder2Previous;
    float m_encoderLGain;
    float m_encoderRGain;
    bool m_is1EncoderRight;
};

#endif /* ENCODERS_H_ */
