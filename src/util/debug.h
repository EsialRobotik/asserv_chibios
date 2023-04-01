/*
 * debug.h
 *
 *  Created on: Apr 1, 2023
 *      Author: pmx
 */

#ifndef SRC_UTIL_DEBUG_H_
#define SRC_UTIL_DEBUG_H_

#define DEBUG_PRINT 1

#if DEBUG_PRINT == 1
	#include "hal.h"
	#include "USBStream.h"
	#include <chprintf.h>
	extern BaseSequentialStream *outputStream;
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



#endif /* SRC_UTIL_DEBUG_H_ */
