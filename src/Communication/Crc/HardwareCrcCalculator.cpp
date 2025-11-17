#include "HardwareCrcCalculator.h"


static const CRCConfig crc32_config = {
    .poly_size         = 32,
    .poly              = 0x4C11DB7,
    .initial_val       = 0xFFFFFFFF,
    .final_val         = 0xffffffff,
    .reflect_data      = 1,
    .reflect_remainder = 1
   };

HardwareCrc32Calculator::HardwareCrc32Calculator(CRCDriver *crcDriver)
{
    m_crcDriver = crcDriver;
    crcAcquireUnit(m_crcDriver);
    crcStart(m_crcDriver, &crc32_config);
    crcReleaseUnit(m_crcDriver);
}


uint32_t HardwareCrc32Calculator::compute(const void *buffer, uint32_t len)
{
    crcAcquireUnit(m_crcDriver);
    crcReset(m_crcDriver);
    uint32_t res = crcCalc(m_crcDriver, len, buffer);
    crcReleaseUnit(m_crcDriver);
    return res;
}