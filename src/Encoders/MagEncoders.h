#ifndef SRC_ENCODERS_MAGENCODERS_CPP_
#define SRC_ENCODERS_MAGENCODERS_CPP_

#include "Encoder.h"

class MagEncoders: public Encoders
{
public:
    MagEncoders(bool is1EncoderRight, bool invertEncoderR = false, bool invertEncoderL = false);
    virtual ~MagEncoders();

    void init();
    void start();
    void stop();

    void getEncodersTotalCount(int32_t *encoderRight, int32_t *encoderLeft);

    virtual void getValues(int16_t *encoderRight, int16_t *encoderLeft);

private:
    bool m_invertEncoderL;
    bool m_invertEncoderR;
    int32_t m_encoderLSum;
    int32_t m_encoderRSum;
    int16_t m_encoder1Previous;
    int16_t m_encoder2Previous;
    bool m_is1EncoderRight;
};



#endif /* SRC_ENCODERS_MAGENCODERS_CPP_ */
