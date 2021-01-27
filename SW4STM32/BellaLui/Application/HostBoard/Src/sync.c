/*
 * sync.c
 *
 *  Created on: 16 Feb 2020
 *      Author: Arion
 */

#include "sync.h"
#include "debug/console.h"

#include <cmsis_os.h>
#include <stm32f4xx_hal.h>


#define TICK_PERIOD (1000 / 20) // 20Hz
#define NUM_SYNCHRONIZABLES 16

static uint32_t last_update[NUM_SYNCHRONIZABLES];
static volatile uint32_t priv_begin = 0;

void sync_logic(uint16_t period) {
	TaskHandle_t handle = xTaskGetCurrentTaskHandle();
	uint8_t* task_name = (uint8_t*) pcTaskGetName(handle);
	uint8_t identifier = task_name[0] - 0x30;

	uint32_t time = HAL_GetTick();
	int32_t delta = period - (time - last_update[identifier]);

	if(delta > 0) {
		osDelay(delta / portTICK_PERIOD_MS);
	}

	if(identifier < NUM_SYNCHRONIZABLES) {
		last_update[identifier] = time;
	}
}

bool do_privileged_io() {
	uint32_t time = HAL_GetTick();

	if(time - priv_begin < 500) { // If no deadlock
		return false;
	} else {
		priv_begin = time;
		return true;
	}
}

bool end_privileged_io() {
	priv_begin = 0;
	return true;
}
