/*
 * threads.h
 *
 *  Created on: 11 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_THREADS_H_
#define APPLICATION_HOSTBOARD_INC_THREADS_H_

#define OS_STKCHECK
#define LED
#define SENSOR_BOARD


#ifdef GPS_BOARD
#define CAN_ID CAN_ID_GPS_BOARD
#define BOARD_LED_R (0)
#define BOARD_LED_G (100)
#define BOARD_LED_B (0)
#define GPS
#define FLASH_LOGGING
#endif

#ifdef SENSOR_BOARD
#define CAN_ID CAN_ID_SENSOR_BOARD
#define SENSOR
#define KALMAN
#define ROCKET_FSM
#define FLASH_LOGGING
//#define CERNIER_LEGACY_DATA
//#define SDCARD
#endif

#ifdef TELEMETRY_BOARD
#define CAN_ID CAN_ID_TELEMETRY_BOARD
#define BOARD_LED_R (80)
#define BOARD_LED_G (50)
#define BOARD_LED_B (0)
#define XBEE
#define FLASH_LOGGING
#endif

#ifdef AIRBRAKES_BOARD
#define CAN_ID CAN_ID_AIBRAKE_BOARD
#define BOARD_LED_R (100)
#define BOARD_LED_G (0)
#define BOARD_LED_B (100)
#define AB_CONTROL
#define GPS
#define FLASH_LOGGING
#endif

#ifdef DEBUG_BOARD
#define CAN_ID CAN_ID_DEBUG_BOARD
#define BOARD_LED_R (50)
#define BOARD_LED_G (50)
#define BOARD_LED_B (50)
#define FLASH_LOGGING
#define ROCKET_FSM
#define XBEE
#endif

#ifdef FLASH_DUMP_BOARD
#define CAN_ID CAN_ID_DEBUG_BOARD
#define BOARD_LED_R (50)
#define BOARD_LED_G (50)
#define BOARD_LED_B (50)
#define FLASH_LOGGING
#endif

#ifdef PROPULSION_BOARD
#define CAN_ID CAN_ID_PROPULSION_BOARD
#define BOARD_LED_R (50)
#define BOARD_LED_G (50)
#define BOARD_LED_B (50)
#define PRESSURE_MONITORING
//todo #define MOTOR_VALVE_CONTROL
#endif

#ifdef XBEE
#include <telemetry/xbee.h>
#endif

#ifdef SENSOR
#include <Sensors/SensorAcquisition.h>
#endif

#ifdef KALMAN
#include <kalman/tiny_ekf.h>
#endif

#ifdef GPS
#include <sensors/GPS_board.h>
#endif

#ifdef AB_CONTROL
#include <airbrakes/airbrake.h>
#include <misc/state_machine.h>
#endif

#ifdef ROCKET_FSM
#include <misc/state_manager.h>
#endif

#ifdef PRESSURE_MONITORING
#include <propulsion/pressure_monitor.h>
#endif

#ifdef FLASH_LOGGING
#include <storage/flash_logging.h>
#endif

#include <storage/heavy_io.h>


#ifdef DEBUG_TASK
#include <Sensors/DataStructures.h>
#endif


void create_semaphores();
void create_threads();


#endif /* APPLICATION_HOSTBOARD_INC_THREADS_H_ */
