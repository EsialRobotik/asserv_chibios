#ifndef SRC_ENCODERS_MAGENCODERS_H_
#define SRC_ENCODERS_MAGENCODERS_H_

#include "Encoder.h"
#include "ch.h"
#include "hal.h"
#include "hal_streams.h"
#include "ams_as5048b.h"

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

    MagEncoders(I2cPinInit *i2cPins, bool is1EncoderRight, bool invertEncoderRight = false, bool invertEncoderLeft = false, uint32_t i2cFrequency=100000);
    virtual ~MagEncoders();

    void init();
    void start();
    void stop();

    void getEncodersTotalCount(int32_t *sumEncoderRight, int32_t *sumEncoderLeft);

    virtual void getValues(float *deltaEncoderRight, float *deltaEncoderLeft);

    void getValuesStatus(uint16_t *encoderRight, uint16_t *encoderLeft, uint8_t *agcR, uint8_t *agcL, uint8_t *diagR,
            uint8_t *diagL, uint16_t *magR, uint16_t *magL);
private:
    I2CConfig m_i2cconfig;
    I2cPinInit m_i2cPinConf;
    AMS_AS5048B m_mysensor1;
    AMS_AS5048B m_mysensor2;

    bool m_invertEncoderL;
    bool m_invertEncoderR;
    int32_t m_encoderLSum;
    int32_t m_encoderRSum;
    float m_encoder1Previous;
    float m_encoder2Previous;

    bool m_is1EncoderRight;

};

#endif /* SRC_ENCODERS_MAGENCODERS_H_ */
