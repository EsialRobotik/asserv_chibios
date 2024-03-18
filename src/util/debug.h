
#ifndef SRC_UTIL_DEBUG_H_
#define SRC_UTIL_DEBUG_H_

#include "hal.h"
#include <chprintf.h>

#define DEBUG_PRINT 0
#define DEBUG_PRINT_DEADLINE 0
#define DEBUG_PRINT_BLOCKED 0

extern BaseSequentialStream *outputStream;

#if DEBUG_PRINT == 1
          #define debug1(a) chprintf(outputStream,a)
          #define debug2(a,b) chprintf(outputStream,a,b)
          #define debug3(a,b,c) chprintf(outputStream,a,b,c)
          #define debug4(a,b,c,d) chprintf(outputStream,a,b,c,d)
#else
          #define debug1(a)
          #define debug2(a,b)
          #define debug3(a,b,c)
          #define debug4(a,b,c,d)
#endif

inline void blink_error(int dutycycle)
{
    int blink = 0 ;
    while(true)
    {
        if (!(blink % dutycycle))
        {
            palSetPad(GPIOA, GPIOA_ARD_D8);
        }else
        {
            palClearPad(GPIOA, GPIOA_ARD_D8);
        }
        chThdSleepMilliseconds(10);
        blink++;
    }
}



#endif /* SRC_UTIL_DEBUG_H_ */

