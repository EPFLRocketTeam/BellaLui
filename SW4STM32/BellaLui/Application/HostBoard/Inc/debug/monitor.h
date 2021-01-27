/*
 * monitor.h
 *
 *  Created on: 5 Sep 2020
 *      Author: arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_DEBUG_MONITOR_H_
#define APPLICATION_HOSTBOARD_INC_DEBUG_MONITOR_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_MONITORS 16

#define SENSOR_MONITOR 		  0
#define STATE_MONITOR  		  1
#define KALMAN_MONITOR 		  2
#define FLASH_MONITOR  		  3
#define CAN_MONITOR    		  4
#define TELEMETRY_MONITOR     5
#define GPS_MONITOR 		  6
#define AIRBRAKES_MONITOR 	  7
#define PROPULSION_MONITOR 	  8

bool enter_monitor(uint8_t id);
bool exit_monitor(uint8_t id);

void enable_monitor(uint8_t id, uint8_t location, uint8_t refresh_rate);
void disable_monitor(uint8_t id);

#endif /* APPLICATION_HOSTBOARD_INC_DEBUG_MONITOR_H_ */
