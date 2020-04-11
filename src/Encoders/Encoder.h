#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

#include <cstdint>

class Encoders
{
public:
    virtual ~Encoders()
    {
    }

    virtual void getValues(int16_t *deltaEncoderRight, int16_t *deltaEncoderLeft) = 0;
};

#endif /* SRC_ENCODER_H_ */
