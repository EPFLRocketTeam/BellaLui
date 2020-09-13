/*
 * profiler.h
 *
 *  Created on: 24 Aug 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_DEBUG_PROFILER_H_
#define APPLICATION_HOSTBOARD_INC_DEBUG_PROFILER_H_

#include <stdint.h>

void start_profiler(uint8_t identifier_addend); // The resulting task id is incremented by the provided value
void end_profiler();
void enable_profiler();
void disable_profiler();

#endif /* APPLICATION_HOSTBOARD_INC_DEBUG_PROFILER_H_ */
