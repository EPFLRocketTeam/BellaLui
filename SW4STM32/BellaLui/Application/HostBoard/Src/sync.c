/*
 * sync.c
 *
 *  Created on: 16 Feb 2020
 *      Author: Arion
 */

#include <cmsis_os.h>
#include <debug/console.h>
#include <stm32f4xx_hal.h>
#include <sync.h>


#define TICK_PERIOD (1000 / 20) // 20Hz
#define NUM_SYNCHRONIZABLES 8

static uint32_t last_update[NUM_SYNCHRONIZABLES];


void sync_logic(uint8_t identifier) {
	uint32_t time = HAL_GetTick();
	int32_t delta = TICK_PERIOD - (time - last_update[identifier]);

	if(delta > 0) {
		osDelay(delta / portTICK_PERIOD_MS);
	}

	last_update[identifier] = time;
}
