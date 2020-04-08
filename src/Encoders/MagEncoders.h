#ifndef SRC_ENCODERS_MAGENCODERS_CPP_
#define SRC_ENCODERS_MAGENCODERS_CPP_

#include "Encoder.h"
#include "ch.h"
#include "hal.h"
#include "ams_as5048b.h"


// Address depending on the two DIL switches
#define AS5048B_ADDR(a2,a1)  (uint8_t)(0x40 | ( a2 ? 0x2 : 0 ) | ( a1 ? 0x1 : 0 ))




class MagEncoders: public Encoders
{
public:

    struct I2cPinInit
    {
        stm32_gpio_t* GPIObaseSCL;
        uint8_t pinNumberSCL;
        stm32_gpio_t* GPIObaseSDA;
        uint8_t pinNumberSDA;
    };

    MagEncoders(I2cPinInit pins, bool is1EncoderRight, bool invertEncoderRight = false, bool invertEncoderLeft = false);
    virtual ~MagEncoders();

    void init();
    void start();
    void stop();

    void getEncodersTotalCount(int32_t *encoderRight, int32_t *encoderLeft);

    virtual void getValues(int16_t *encoderRight, int16_t *encoderLeft);

private:
    AMS_AS5048B * m_mysensor;
    I2CConfig m_i2cconfig;
    I2cPinInit m_i2cPinConf;
    /*
    bool m_invertEncoderL;
    bool m_invertEncoderR;
    int32_t m_encoderLSum;
    int32_t m_encoderRSum;
    int16_t m_encoder1Previous;
    int16_t m_encoder2Previous;
    bool m_is1EncoderRight;*/
};

#endif /* SRC_ENCODERS_MAGENCODERS_CPP_ */
