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
#include <stdbool.h>

#define NUM_PROFILERS 16
#define MAX_PROFILER_DEPTH 8
#define TIME_BETWEEN_UPDATES 1000

struct ThreadProfiler {
	uint8_t depth;
	uint32_t last_update;
	int8_t depth_to_display;
	uint32_t timestamps[MAX_PROFILER_DEPTH];
	uint8_t measurement_id[MAX_PROFILER_DEPTH];
};

struct ThreadProfiler profilers[NUM_PROFILERS];

static volatile bool profiling_requested;
static bool profiling;

void start_profiler(uint8_t measurement_id) {
	TaskHandle_t handle = xTaskGetCurrentTaskHandle();
	uint8_t* task_name = (uint8_t*) pcTaskGetName(handle);
	uint8_t task_id = task_name[0] - 0x30;

	if(task_id < NUM_PROFILERS) {
		struct ThreadProfiler* profiler = &profilers[task_id];

		profiler->measurement_id[profiler->depth] = measurement_id;
		profiler->measurement_id[profiler->depth + 1] = 0;
		profiler->timestamps[profiler->depth] = HAL_GetTick();

		profiler->depth++;

		profiler->measurement_id[profiler->depth] = 0;

		if(profiler->depth >= MAX_PROFILER_DEPTH) {
			profiler->depth = MAX_PROFILER_DEPTH - 1;
			rocket_log("Attempt from thread %s to exceed the maximum profiler nesting depth!\n", task_name);
		}
	} else {
		rocket_log("Unable to profile thread %s\n", task_name);
	}
}

static void _spaces(uint8_t count) {
	while(count--) {
		rocket_log(" ");
	}
}

void end_profiler() {
	TaskHandle_t handle = xTaskGetCurrentTaskHandle();
	uint8_t* task_name = (uint8_t*) pcTaskGetName(handle);
	uint8_t task_id = task_name[0] - 0x30;

	if(task_id < NUM_PROFILERS) {
		struct ThreadProfiler* profiler = &profilers[task_id];

		profiler->depth--;

		uint32_t time = HAL_GetTick();
		uint32_t diff = time - profiler->timestamps[profiler->depth]; // Should not overflow
		profiler->timestamps[profiler->depth] = 0;

		if(profiler->depth == profiler->depth_to_display) {
			uint8_t measurement_id = (uint8_t) profiler->measurement_id[profiler->depth];

			if(profiling) {
				rocket_log("\e7\x1b[%u;%uH", 1 + (uint32_t) measurement_id, (uint32_t) (20 * task_id));
				_spaces(profiler->depth);
				rocket_log(" (%u) %u.5 %-9s\e8", (uint32_t) measurement_id, (uint32_t) diff, "ms");
			}
		}

		if(profiler->depth == 0) {
			if(profiler->depth_to_display == 0) {  // Update layer per layer
				if(profiling) {
					rocket_log_release();
				}
			} else if(time - profiler->last_update > TIME_BETWEEN_UPDATES) {
				profiler->last_update = time - diff;

				if(profiling_requested) {
					profiling_requested = false;
					profiling = true;
				}

				if(profiling) {
					rocket_log_lock();
					rocket_log("\e7\x1b[0;%dH %-16s |\e8", 20 * task_id, task_name);

					profiler->depth_to_display = MAX_PROFILER_DEPTH; // Magic number. Signals an update request.
				}
			}

			profiler->depth_to_display--;
		}
	}
}

void enable_profiler() {
	profiling_requested = true;

	for(uint8_t i = 0; i < NUM_PROFILERS; i++) {
		profilers[i].depth_to_display = -1;
	}
}

void disable_profiler() {
	profiling = false;
}
