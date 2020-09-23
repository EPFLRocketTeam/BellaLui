/*
 * CAN_communication.h
 *
 *  Created on: Feb 23, 2019
 *      Author: Tim Lebailly
 */

#ifndef CAN_COMMUNICATION_H_
#define CAN_COMMUNICATION_H_

#include <can_reception.h>
#include "stm32f4xx_hal.h"
#include <string.h>


typedef struct
{
    uint32_t data;
    uint8_t id;
    uint32_t timestamp;
    uint32_t id_CAN;
} CAN_msg;

// Define all the data ID's
#define DATA_ID_PRESSURE 0 // hPa
#define DATA_ID_ACCELERATION_X 1 // milli-g
#define DATA_ID_ACCELERATION_Y 2 // milli-g
#define DATA_ID_ACCELERATION_Z 3 // milli-g
#define DATA_ID_GYRO_X 4 // mrps
#define DATA_ID_GYRO_Y 5 // mrps
#define DATA_ID_GYRO_Z 6 // mrps

#define DATA_ID_GPS_HDOP      7 // mm
#define DATA_ID_GPS_LAT       8 // udeg
#define DATA_ID_GPS_LONG      9 // udeg
#define DATA_ID_GPS_ALTITUDE 10 // cm
#define DATA_ID_GPS_SATS     11 // #

#define DATA_ID_TEMPERATURE 12 // cDegC
#define DATA_ID_CALIB_PRESSURE 13 // Pa

#define DATA_ID_AB_STATE   16 // enum
#define DATA_ID_AB_INC     17 // [-]
#define DATA_ID_AB_AIRSPEED 18 // mm/s
#define DATA_ID_AB_ALT     19 // m

#define DATA_ID_KALMAN_STATE 38 // enum
#define DATA_ID_KALMAN_X     40 // m
#define DATA_ID_KALMAN_Y     41 // m
#define DATA_ID_KALMAN_Z     42 // m
#define DATA_ID_KALMAN_VX    43 // mm/s
#define DATA_ID_KALMAN_VY    44 // mm/s
#define DATA_ID_KALMAN_VZ    45 // mm/s
#define DATA_ID_KALMAN_YAW   46 // mrad
#define DATA_ID_KALMAN_PITCH 47 // mrad
#define DATA_ID_KALMAN_ROLL  48 // mrad

#define DATA_ID_STATE 50 // enum


#define DATA_ID_START_VALVE_OPERATION 80
#define DATA_ID_START_FUELING 81
#define DATA_ID_STOP_FUELING 82
#define DATA_ID_START_HOMING 83
#define DATA_ID_ABORT 84



#define DATA_ID_PROP_PRESSURE1 85 // enum
#define DATA_ID_PROP_PRESSURE2 86 // enum
#define DATA_ID_PROP_TEMPERATURE1 87 // enum
#define DATA_ID_PROP_TEMPERATURE2 88 // enum
#define DATA_ID_PROP_TEMPERATURE3 89 // enum
#define DATA_ID_PROP_STATUS 90 // enum
#define DATA_ID_PROP_MOTOR_POSITION 91 // enum

#define DATA_ID_SHELL_CONTROL  200
#define DATA_ID_SHELL_INPUT    201
#define DATA_ID_SHELL_OUTPUT   202

// Define all the board ID's (lower means higher priority for CAN protocol)
#define CAN_ID_SENSOR_BOARD 0
#define CAN_ID_GPS_BOARD 1
#define CAN_ID_TELEMETRY_BOARD 2
#define CAN_ID_AIBRAKE_BOARD 3
#define CAN_ID_DEBUG_BOARD 6
#define CAN_ID_DEFAULT 7
#define CAN_ID_PROPULSION_BOARD 5

#define MAX_BOARD_ID 7 // used to implement redundant info in CAN_handling
#define MAX_BOARD_NUMBER (MAX_BOARD_ID+1)

void CAN_Config(uint32_t id);
void can_setFrame(uint32_t data, uint8_t data_id, uint32_t timestamp);
void can_addMsg(CAN_msg msg);
uint32_t can_msgPending();
CAN_msg can_readBuffer();

uint8_t get_board_id();


#endif /* CAN_COMMUNICATION_H_ */
