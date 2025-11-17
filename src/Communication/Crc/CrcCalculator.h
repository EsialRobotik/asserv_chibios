#ifndef SRC_CRCCALCULATOR_INTERFACE_H_
#define SRC_CRCCALCULATOR_INTERFACE_H_

#include <cstdint>

class Crc32Calculator
{
public:
    virtual ~Crc32Calculator()
    {
    }

    virtual uint32_t compute(const void *buffer, uint32_t len) = 0;
};

#endif /* SRC_CRCCALCULATOR_INTERFACE_H_ */
