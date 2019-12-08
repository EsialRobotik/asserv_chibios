#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

#include <cstdint>

class Encoders
{
    public:
        virtual ~Encoders() {}

    	virtual void init() = 0;
    	virtual void start() = 0;
    	virtual void stop() = 0;

    	virtual void getValuesAndReset(int16_t *encoder1, int16_t *encoder2) = 0;
};

#endif /* SRC_ENCODER_H_ */
