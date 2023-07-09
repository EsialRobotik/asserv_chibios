#ifndef ENCODERS__SIMULATION_H_
#define ENCODERS__SIMULATION_H_

#include "Encoder.h"

class EncoderSimuation: public Encoders
{
public:

    EncoderSimuation();
    virtual ~EncoderSimuation();

    virtual void getValues(float *deltaEncoderRight, float *deltaEncoderLeft);

private:
    float m_encoderR;
    bool m_encoderL;
};

#endif /* ENCODERS__SIMULATIONH_ */
