#ifndef SRC_HARDWARECRCCALCULATOR_INTERFACE_H_
#define SRC_HARDWARECRCCALCULATOR_INTERFACE_H_

#include "CrcCalculator.h"
#include "ch.h"
#include "hal.h"


class HardwareCrc32Calculator : public Crc32Calculator
{
    public:
        explicit HardwareCrc32Calculator(CRCDriver *crcDriver);


        virtual uint32_t compute(const void *buffer, uint32_t len);

    private:
        CRCDriver *m_crcDriver;
};

#endif /* SRC_BLOCKINGDETECTOR_SPEEDERRORBLOCKINGDETECTOR_H_ */
