#include "SoftwareCrcCalculator.h"
#include "cppCrc.h"

SoftwareCrc32Calculator::SoftwareCrc32Calculator()
{
}


uint32_t SoftwareCrc32Calculator::compute(const void *buffer, uint32_t len)
{
    return CRC32::CRC32::calc((const uint8_t*)buffer, len);
}