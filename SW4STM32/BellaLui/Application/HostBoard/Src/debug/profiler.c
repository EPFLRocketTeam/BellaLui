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
#define TICKS_PER_SAMPLE 64

struct ThreadProfiler {
	uint8_t depth;
	uint8_t samples;
	uint32_t timestamps[MAX_PROFILER_DEPTH];
	uint8_t measurement_id[MAX_PROFILER_DEPTH];
};

struct ThreadProfiler profilers[NUM_PROFILERS];

void start_profiler(uint8_t identifier_addend) {
	TaskHandle_t handle = xTaskGetCurrentTaskHandle();
	uint32_t taskID = uxTaskGetTaskNumber(handle);

	if(taskID < NUM_PROFILERS) {
		struct ThreadProfiler* profiler = &profilers[taskID];

		profiler->measurement_id[profiler->depth] += identifier_addend;
		profiler->measurement_id[profiler->depth + 1] = 0;
		profiler->timestamps[profiler->depth] = HAL_GetTick();

		profiler->depth++;

		profiler->measurement_id[profiler->depth] = 0;

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
		rocket_log(" > ");
	}
}

void end_profiler() {
	TaskHandle_t handle = xTaskGetCurrentTaskHandle();
	uint32_t taskID = uxTaskGetTaskNumber(handle);

	if(taskID < NUM_PROFILERS) {
		struct ThreadProfiler* profiler = &profilers[taskID];

		profiler->depth--;

		uint32_t time = HAL_GetTick();
		uint32_t diff = time - profiler->timestamps[profiler->depth]; // Should not overflow
		profiler->timestamps[profiler->depth] = 0;

		if(profiler->samples > TICKS_PER_SAMPLE) {
			uint8_t measurement_id = (uint8_t) profiler->measurement_id[profiler->depth];
			_tabs(profiler->depth);

			for(uint8_t i = 0; i < profiler->depth; i++) {
				profiler->timestamps[i] += 1; // One rocket_log takes 1.7 ms

				if(time % 3 == 0 || time % 3 == 1) {
					profiler->timestamps[i] += 1; // Suppose time mod 2 is uniformly distributed
				}
			}

			rocket_log("Thread %s took %d.5 ms to perform task %u.\n", pcTaskGetName(handle), (uint32_t) diff, measurement_id);

		}

		if(profiler->depth == 0) {
			if(profiler->samples > TICKS_PER_SAMPLE) {
				profiler->samples = 0;
				rocket_log("\n ------------------------------\n\n\x1b[40;0H");
				rocket_log_release();
			} else if(profiler->samples == TICKS_PER_SAMPLE) {
				rocket_log_lock();
				rocket_log("\x1b[15;0H\x1b[1J\x1b[H\n"); // Reset cursor
				rocket_log(" ---------- Profiler ----------\n\n");
			}

			profiler->samples++;
		}
	}
}
