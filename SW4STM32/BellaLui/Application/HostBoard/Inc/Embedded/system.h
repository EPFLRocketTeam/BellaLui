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
#define pvPortMalloc malloc
#define vPortFree free
#else
#include <cmsis_os.h>
#endif


#endif /* APPLICATION_HOSTBOARD_SRC_EMBEDDED_MALLOC_H_ */
