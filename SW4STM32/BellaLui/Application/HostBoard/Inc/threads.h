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
//#define TELEMETRY_BOARD
//#define VALVE_BOARD
//#define CODE_BOARD
#define SENSOR_TELEMETRY_BOARD



#ifdef GPS_BOARD
#define CAN_ID CAN_ID_GPS_BOARD
#define GPS
#define FLASH_LOGGING
#define BOARD_LED_R (0)
#define BOARD_LED_G (100)
#define BOARD_LED_B (0)
#endif

#ifdef SENSOR_BOARD
#define CAN_ID CAN_ID_MAIN_BOARD
#define SENSOR
#define KALMAN
#define ROCKET_FSM
#define FLASH_LOGGING
#define BOARD_LED_R (0)
#define BOARD_LED_G (100)
#define BOARD_LED_B (100)
//#define CERNIER_LEGACY_DATA
//#define SDCARD
#endif

#ifdef TELEMETRY_BOARD
#define CAN_ID CAN_ID_TELEMETRY_BOARD
#define XBEE
//#define SDCARD
#define BOARD_LED_R (80)
#define BOARD_LED_G (50)
#define BOARD_LED_B (0)
#endif

#ifdef AIRBRAKES_BOARD
#define BOARD_LED_R (100)
#define BOARD_LED_G (0)
#define BOARD_LED_B (100)
#define AB_CONTROL
#define GPS
//#define FLASH_LOGGING
//#define CAN_LED
#define CAN_ID CAN_ID_AIBRAKE_BOARD
//#define SDCARD
#endif

#ifdef DEBUG_BOARD
#define BOARD_LED_R (50)
#define BOARD_LED_G (50)
#define BOARD_LED_B (50)
#define CAN_ID CAN_ID_DEBUG_BOARD
#define FLASH_LOGGING
//#define SDCARD
#endif

#ifdef FLASH_DUMP_BOARD
#define BOARD_LED_R (50)
#define BOARD_LED_G (50)
#define BOARD_LED_B (50)
#define CAN_ID CAN_ID_DEBUG_BOARD
#define FLASH_LOGGING
#endif

#ifdef PROPULSION_BOARD
#define BOARD_LED_R (50)
#define BOARD_LED_G (50)
#define BOARD_LED_B (50)
#define CAN_ID CAN_ID_PROPULSION_BOARD
#define PRESSURE_MONITORING
//todo #define MOTOR_VALVE_CONTROL
#endif

//GSE Boards
#ifdef VALVE_BOARD
#define CAN_ID CAN_ID_GSE_VALVE_BOARD
#define VALVE
#define IGNITION
#define IGNITION_1

#define BOARD_LED_R (0)
#define BOARD_LED_G (127)
#define BOARD_LED_B (127)
#endif

#ifdef CODE_BOARD
#define CAN_ID CAN_ID_GSE_CODE_BOARD
#define SECURITY_CODE
#define IGNITION
#define DISCONNECT

#define BOARD_LED_R (0)
#define BOARD_LED_G (255)
#define BOARD_LED_B (0)
#endif

#ifdef SENSOR_TELEMETRY_BOARD
#define CAN_ID CAN_ID_GSE_SENSOR_TELEMETRY_BOARD
#define SENSOR_TELEMETRY
#define XBEE

//########################################## TEST ##########################################
//#define VALVE
//########################################## TEST ##########################################

//#define IGNITION
//#define IGNITION_2

#define BOARD_LED_R (0)
#define BOARD_LED_G (0)
#define BOARD_LED_B (255)
#endif

#ifdef XBEE
#include <telemetry/xbee.h>
#endif

#ifdef SENSOR
#include <sensors/sensor_board.h>
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
#include <misc/state_machine.h>
#endif

#ifdef PRESSURE_MONITORING
#include <propulsion/pressure_monitor.h>
#endif

//GSE Defs
#ifdef VALVE
#include <GSE/valve.h>
#endif

#ifdef IGNITION
#include <GSE/ignition.h>
#endif

#ifdef SECURITY_CODE
#include <GSE/code.h>
#endif

#ifdef SENSOR_TELEMETRY
#include <GSE/sensor_telemetry.h>
#endif

void create_semaphores();
void create_threads();


#endif /* APPLICATION_HOSTBOARD_INC_THREADS_H_ */
