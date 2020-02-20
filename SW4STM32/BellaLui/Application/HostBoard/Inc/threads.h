/*
 * threads.h
 *
 *  Created on: 11 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_THREADS_H_
#define APPLICATION_HOSTBOARD_INC_THREADS_H_



#define DEBUG
#define SENSOR_BOARD


#define OS_STKCHECK
#define LED
#define CAN_LED


#ifdef GPS_BOARD
#define CAN_ID CAN_ID_TELEMETRY_BOARD
#define GPS
#define KALMAN
#define BOARD_LED_R (0)
#define BOARD_LED_G (100)
#define BOARD_LED_B (0)
#endif

#ifdef SENSOR_BOARD
#define CAN_ID CAN_ID_MAIN_BOARD
#define SENSOR
<<<<<<< HEAD
#define KALMAN
=======
//#define KALMAN
>>>>>>> branch 'master' of https://github.com/EPFLRocketTeam/BellaLui.git
//#define FLASH_LOGGING
#define BOARD_LED_R (0)
#define BOARD_LED_G (100)
#define BOARD_LED_B (0)
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

#ifdef AIRBRAKE_BOARD
#define BOARD_LED_R (100)
#define BOARD_LED_G (0)
#define BOARD_LED_B (100)
#define AB_CONTROL
#define ROCKET_FSM
#define SENSOR
//#define CAN_LED
#define CAN_ID CAN_ID_AIBRAKE_BOARD
//#define SDCARD
#endif

#ifdef DEBUG_BOARD
#define BOARD_LED_R (50)
#define BOARD_LED_G (50)
#define BOARD_LED_B (50)
#define CAN_ID CAN_ID_DEBUG_BOARD
#define SENSOR
//#define SDCARD
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
#endif

#ifdef ROCKET_FSM
#include <misc/state_machine.h>
#endif

void create_semaphores();
void create_threads();


#endif /* APPLICATION_HOSTBOARD_INC_THREADS_H_ */
