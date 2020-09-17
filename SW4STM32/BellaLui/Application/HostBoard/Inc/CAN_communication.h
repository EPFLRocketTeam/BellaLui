/*
 * CAN_communication.h
 *
 *  Created on: Feb 23, 2019
 *      Author: Tim Lebailly
 */

#ifndef CAN_COMMUNICATION_H_
#define CAN_COMMUNICATION_H_

#include "stm32f4xx_hal.h"
#include <string.h>

#include "../../HostBoard/Inc/CAN_handling.h"

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

//GSE Rx Data
#define DATA_ID_IGNITION 51
#define DATA_ID_ORDER 52
#define DATA_ID_GST_CODE 53

//GSE Tx Data
#define DATA_ID_GSE_CODE 54
#define DATA_ID_FILL_VALVE_STATE 56
#define DATA_ID_PURGE_VALVE_STATE 57
#define DATA_ID_HOSE_DISCONNECT_STATE 58
#define DATA_ID_MAIN_IGNITION_STATE 59
#define DATA_ID_SEC_IGNITION_STATE 60

//GSE Sensor Data
#define DATA_ID_IGNITION_CURRENT_1 61
#define DATA_ID_IGNITION_CURRENT_2 62
#define DATA_ID_DISCONNECT_CURRENT 63
#define DATA_ID_IN_HOSE_PRESSURE 64
#define DATA_ID_IN_HOSE_TEMPERATURE 65
#define DATA_ID_TANK_TEMPERATURE 66
#define DATA_ID_ROCKET_WEIGHT 67

//Propulsion Data
#define DATA_ID_MOTOR_PRESSURE 55


// Define all the board ID's (lower means higher priority for CAN protocol)
#define CAN_ID_MAIN_BOARD 0
#define CAN_ID_GPS_BOARD 1
#define CAN_ID_TELEMETRY_BOARD 2
#define CAN_ID_AIBRAKE_BOARD 3
#define CAN_ID_DEBUG_BOARD 6
#define CAN_ID_DEFAULT 7
#define CAN_ID_PROPULSION_BOARD 5
#define CAN_ID_VALVE_BOARD 8
#define CAN_ID_CODE_BOARD 10
#define CAN_ID_SENSOR_TELEMETRY_BOARD 9

#define MAX_BOARD_ID 9 // used to implement redundant info in CAN_handling
#define MAX_BOARD_NUMBER (MAX_BOARD_ID+1)

void CAN_Config(uint32_t id);
void can_setFrame(uint32_t data, uint8_t data_id, uint32_t timestamp);

uint32_t can_msgPending();
CAN_msg can_readBuffer();


#endif /* CAN_COMMUNICATION_H_ */
