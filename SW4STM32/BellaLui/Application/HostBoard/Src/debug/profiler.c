/*
 * profiler.c
 *
 *  Created on: 24 Aug 2020
 *      Author: Arion
 */

#include "debug/profiler.h"
#include "debug/console.h"

#include <cmsis_os.h>
#include <stm32f4xx.h>

#define NUM_PROFILERS 16
#define MAX_PROFILER_DEPTH 8
#define NUM_SAMPLES 64

struct ThreadProfiler {
	uint8_t depth;
	uint8_t samples;
	uint64_t integral[MAX_PROFILER_DEPTH];
	uint64_t timestamps[MAX_PROFILER_DEPTH];
	uint8_t measurement_id[MAX_PROFILER_DEPTH];

};

struct ThreadProfiler profilers[NUM_PROFILERS];

void start_profiler(uint8_t identifier_addend) {
	TaskHandle_t handle = xTaskGetCurrentTaskHandle();
	uint32_t taskID = uxTaskGetTaskNumber(handle);

	if(taskID < NUM_PROFILERS) {
		struct ThreadProfiler* profiler = &profilers[taskID];

		profiler->measurement_id[profiler->depth] += identifier_addend;
		profiler->timestamps[profiler->depth++] = HAL_GetTick();

		if(profiler->depth >= MAX_PROFILER_DEPTH) {
			profiler->depth = MAX_PROFILER_DEPTH - 1;
			rocket_log("Attempt from thread %s to exceed the maximum profiler nesting depth!\n", pcTaskGetName(handle));
		}
	} else {
		rocket_log("Unable to profile thread %s\n", pcTaskGetName(handle));
	}
}

void _tabs(uint8_t count) {
	while(count--) {
		rocket_log("\t");
	}
}

void end_profiler() {
	TaskHandle_t handle = xTaskGetCurrentTaskHandle();
	uint32_t taskID = uxTaskGetTaskNumber(handle);

	if(taskID < NUM_PROFILERS) {
		struct ThreadProfiler* profiler = &profilers[taskID];

		profiler->measurement_id[profiler->depth--] = 0;

		profiler->integral[profiler->depth] += 1000 * (HAL_GetTick() - profiler->timestamps[profiler->depth]); // Should not overflow

		if(profiler->samples > NUM_SAMPLES) {
			_tabs(profiler->depth);
			rocket_log("Thread %s took %d µs to perform task %u.\n", pcTaskGetName(handle), (uint32_t) profiler->integral[profiler->depth] / NUM_SAMPLES, (uint8_t) profiler->measurement_id[profiler->depth]);
			profiler->integral[profiler->depth] = 0;
		}


		if(profiler->depth == 0) {
			if(profiler->samples++ > NUM_SAMPLES) {
				profiler->samples = 0;
				rocket_log("\n");
			}
		}
	}
}
