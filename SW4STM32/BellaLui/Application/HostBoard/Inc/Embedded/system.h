/*
 * malloc.h
 *
 *  Created on: 22 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_SRC_EMBEDDED_MALLOC_H_
#define APPLICATION_HOSTBOARD_SRC_EMBEDDED_MALLOC_H_


#ifdef TESTING

#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#define pvPortMalloc malloc
#define vPortFree free

inline uint32_t HAL_GetTick() {
	struct timespec ts;
	timespec_get(&ts, TIME_UTC);
	return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

#else
#include <cmsis_os.h>
#include "stm32f4xx_hal.h"
#endif


#endif /* APPLICATION_HOSTBOARD_SRC_EMBEDDED_MALLOC_H_ */
