/*
 * monitor.c
 *
 *  Created on: 5 Sep 2020
 *      Author: arion
 */

#include "debug/monitor.h"

#include "debug/console.h"

#include <stm32f4xx_hal.h>


static uint8_t locations[MAX_MONITORS];
static uint8_t monitor_refresh_rate[MAX_MONITORS];
static uint32_t last_update[MAX_MONITORS];


bool enter_monitor(uint8_t id) {
	uint32_t time = HAL_GetTick();

	if(monitor_refresh_rate[id] == 0 || time - last_update[id] < 1000 / monitor_refresh_rate[id]) {
		return false;
	}

	last_update[id] = time;

	rocket_log_lock();

	rocket_log("\e7"); // Save cursor
	rocket_log("\x1b[%u;0H", locations[id]); // Move cursor to monitor location

	return true;
}

bool exit_monitor(uint8_t id) {
	if(!monitor_refresh_rate[id]) {
		return false;
	}

	rocket_log("\e8"); // Restore cursor

	rocket_log_release();

	return true;
}

void enable_monitor(uint8_t id, uint8_t location, uint8_t refresh_rate) {
	locations[id] = location;
	monitor_refresh_rate[id] = refresh_rate;
}

void disable_monitor(uint8_t id) {
	monitor_refresh_rate[id] = 0;
}
