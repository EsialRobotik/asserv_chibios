#ifndef SRC_HARDWARECRCCALCULATOR_INTERFACE_H_
#define SRC_HARDWARECRCCALCULATOR_INTERFACE_H_

#include "CrcCalculator.h"


class SoftwareCrc32Calculator : public Crc32Calculator
{
    public:
        explicit SoftwareCrc32Calculator();


        virtual uint32_t compute(const void *buffer, uint32_t len);

};

#endif /* SRC_BLOCKINGDETECTOR_SPEEDERRORBLOCKINGDETECTOR_H_ */
