/*
 * threads.h
 *
 *  Created on: 11 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_THREADS_H_
#define APPLICATION_HOSTBOARD_INC_THREADS_H_



#define DEBUG
#define TELEMETRY_BOARD


#define OS_STKCHECK
#define LED
#define CAN_LED


#ifdef SENSOR_BOARD
#define CAN_ID CAN_ID_MAIN_BOARD
#define SENSOR
#define FLASH_LOGGING
#define BOARD_LED_R (0)
#define BOARD_LED_G (100)
#define BOARD_LED_B (0)
#endif


#ifdef TELEMETRY_BOARD
#define CAN_ID CAN_ID_TELEMETRY_BOARD
#define XBEE
#define BOARD_LED_R (80)
#define BOARD_LED_G (50)
#define BOARD_LED_B (0)
#endif


#ifdef GPS_BOARD
#define CAN_ID CAN_ID_TELEMETRY_BOARD
#define GPS
#define KALMAN
#define BOARD_LED_R (0)
#define BOARD_LED_G (100)
#define BOARD_LED_B (0)
#endif


#ifdef AIRBRAKES_BOARD
#define CAN_ID CAN_ID_AIBRAKES_BOARD
#define AB_CONTROL
#define ROCKET_FSM
#define BOARD_LED_R (100)
#define BOARD_LED_G (0)
#define BOARD_LED_B (100)
#endif


#ifdef XBEE
#include <telemetry/xbee.h>
#endif

#ifdef SENSOR
#include <sensors/sensor_board.h>
#endif


void create_semaphores();
void create_threads();


#endif /* APPLICATION_HOSTBOARD_INC_THREADS_H_ */
