/*
 * CAN_communication.c
 *
 * If you read a frame and there is a message, the led blinks blue.
 * If you write a frame and it fails, the led blinks red.
 * If you write a frame and if does not fail, the led blinks green.
 *
 *  Created on: Feb 23, 2019
 *      Author: Tim Lebailly
 */
typedef float float32_t;

#include <stdbool.h>
#include "cmsis_os.h"

#include "CAN_communication.h"
#include "main.h"
#include "led.h"
#include "Telemetry/telemetry_handling.h"
#include "airbrake/airbrake.h"
#include "Sensors/GPS_board.h"
#include "Sensors/sensor_board.h"
#include "Misc/datastructs.h"
#include "sd_card.h"
#include "ekf/tiny_ekf.h"
#include "Misc/Common.h"

#define BUFFER_SIZE 128

#define GPS_DEFAULT (-1.0)

IMU_data IMU_buffer[CIRC_BUFFER_SIZE];
BARO_data BARO_buffer[CIRC_BUFFER_SIZE];

float kalman_z  = 0;
float kalman_vz = 0;
int32_t ab_angle = 42;

// wrapper to avoid fatal crashes when implementing redundancy
int board2Idx(uint32_t board) {
	if (board < MAX_BOARD_ID) {
		return board;
	}  else { // invalid board ID
		// to avoid fatal crash return a default value
		return MAX_BOARD_ID;
	}
}

bool handleGPSData(GPS_data data) {
#ifdef XBEE
	return telemetry_handleGPSData(data);
#elif defined(KALMAN)
	if (data.lat < 1e3) {
		return kalman_handleGPSData(data);
	}
#endif
	return false;
}

bool handleIMUData(IMU_data data) {
#ifdef XBEE
	return telemetry_handleIMUData(data);
#elif defined(KALMAN)
	return kalman_handleIMUData(data);
#else
	IMU_buffer[(++currentImuSeqNumber) % CIRC_BUFFER_SIZE] = data;
#endif
	return true;
}

bool handleBaroData(BARO_data data) {
	data.altitude = altitudeFromPressure(data.pressure);

#ifdef CERNIER_LEGACY_DATA
	data.base_pressure = 938.86;
#endif

	if (data.base_pressure > 0) {
		data.base_altitude = altitudeFromPressure(data.base_pressure);
	}

#ifdef XBEE
	return telemetry_handleBaroData(data);
#elif defined(KALMAN)
	return kalman_handleBaroData(data);
#else
	BARO_buffer[(++currentBaroSeqNumber) % CIRC_BUFFER_SIZE] = data;
	currentBaroTimestamp = HAL_GetTick();
#endif
	return false;
}

float can_getAltitude() {
	//return altitude_estimate; // from TK_state_estimation
	return kalman_z;
}

float can_getSpeed() {
	//return air_speed_state_estimate; // from TK_state_estimation
	return kalman_vz;
}

uint8_t can_getState() {
	return currentState;
}

int32_t can_getABangle() {
	return ab_angle;
}

void sendSDcard(CAN_msg msg) {
   static char buffer[BUFFER_SIZE] = {0};
   static uint32_t sdSeqNumber = 0;
   sdSeqNumber++;

   uint32_t id_can = msg.id_CAN;
   uint32_t timestamp = msg.timestamp;
   uint8_t id = msg.id;
   uint32_t data = msg.data;

   sprintf((char*) buffer, "%lu\t%lu\t%d\t%ld\n",
		   sdSeqNumber, HAL_GetTick(), id, (int32_t) data);

   sd_write(buffer, strlen(buffer));
}

void TK_can_reader() {
	// init
	CAN_msg msg;

	IMU_data  imu [MAX_BOARD_NUMBER] = {0};
	BARO_data baro[MAX_BOARD_NUMBER] = {0};
	GPS_data  gps [MAX_BOARD_NUMBER] = {0};
	int total_gps_fixes = 0;
	bool gps_fix [MAX_BOARD_NUMBER] = {0};
	bool new_baro[MAX_BOARD_NUMBER] = {0};
	bool new_imu [MAX_BOARD_NUMBER] = {0};
	bool new_gps [MAX_BOARD_NUMBER] = {0};
	int idx = 0;

	osDelay (500); // Wait for the other threads to be ready

	for (;;)
	{
		while (can_msgPending()) { // check if new data
			msg = can_readBuffer();
			// add to SD card
#ifdef SDCARD
			sendSDcard(msg);
#endif
			idx = board2Idx(msg.id_CAN);

			switch(msg.id) {
			case DATA_ID_PRESSURE:
				baro[idx].pressure = ((float32_t) ((int32_t) msg.data)) / 100; // convert from cPa to hPa
				new_baro[idx] = true; // only update when we get the pressure
				break;
			case DATA_ID_TEMPERATURE:
				baro[idx].temperature = ((float32_t) ((int32_t) msg.data)) / 100; // from to cDegC in DegC
				break;
			case DATA_ID_CALIB_PRESSURE:
				baro[idx].base_pressure = ((float32_t) ((int32_t) msg.data)) / 100; // from cPa to hPa
				break;
			case DATA_ID_ACCELERATION_X:
				imu[idx].acceleration.x = ((float32_t) ((int32_t) msg.data)) / 1000; // convert from m-g to g
				break;
			case DATA_ID_ACCELERATION_Y:
				imu[idx].acceleration.y = ((float32_t) ((int32_t) msg.data)) / 1000;
				break;
			case DATA_ID_ACCELERATION_Z:
				imu[idx].acceleration.z = ((float32_t) ((int32_t) msg.data)) / 1000;
				new_imu[idx] = true;  // only update when we get IMU from Z
				break;
			case DATA_ID_GYRO_X:
				imu[idx].eulerAngles.x = ((float32_t) ((int32_t) msg.data)); // convert from mrps to ???
				break;
			case DATA_ID_GYRO_Y:
				imu[idx].eulerAngles.y = ((float32_t) ((int32_t) msg.data));
				break;
			case DATA_ID_GYRO_Z:
				imu[idx].eulerAngles.z = ((float32_t) ((int32_t) msg.data));
				break;
			case DATA_ID_GPS_HDOP:
				gps[idx].hdop = ((float32_t) ((int32_t) msg.data)) / 1e3; // from mm to m
				if (!gps_fix[idx]) {
					gps_fix[idx] = true;
					total_gps_fixes++;
				}
				break;
			case DATA_ID_GPS_LAT:
				gps[idx].lat = ((float32_t) ((int32_t) msg.data))  / 1e6; // from udeg to deg
				break;
			case DATA_ID_GPS_LONG:
				gps[idx].lon = ((float32_t) ((int32_t) msg.data))  / 1e6; // from udeg to deg
				break;
			case DATA_ID_GPS_ALTITUDE:
				gps[idx].altitude = ((int32_t) msg.data) / 1; // keep in cm
				break;
			case DATA_ID_GPS_SATS:
				gps[idx].sats = ((uint8_t) ((int32_t) msg.data));
				new_gps[idx] = true;
				break;
			case DATA_ID_STATE:
#ifndef ROCKET_FSM // to avoid self loop on board with FSM
				currentState = msg.data;
#endif
				break;
			case DATA_ID_KALMAN_Z:
				kalman_z = ((float32_t) ((int32_t) msg.data))/1e3;
				break;
			case DATA_ID_KALMAN_VZ:
				kalman_vz = ((float32_t) ((int32_t) msg.data))/1e3;
				break;
			case DATA_ID_AB_INC:
				ab_angle = (int32_t) msg.data;
				break;
			}
		}

		// check if new/non-handled full sensor packets are present
		for (int i=0; i<MAX_BOARD_NUMBER ; i++) {
			if (new_gps[i]) {
				// check if the new gps data has a fix
				if (gps[i].altitude == GPS_DEFAULT) { // will make some launch locations impossible (depending on the default value the altitude might be valid)
					if (gps_fix[i]) { // todo: implement timeout on gps_fix if no message received for extended time
						total_gps_fixes--;
						gps_fix[i] = false;
					}
				}

				if (gps_fix[i] || total_gps_fixes<1) { // filter packets
					// only allow packets without fix if there exist globally no gps fixes
					if (handleGPSData(gps[i])) { // handle packet
						// reset all the data
						gps[i].hdop     = GPS_DEFAULT;
						gps[i].lat      = GPS_DEFAULT;
						gps[i].lon      = GPS_DEFAULT;
						gps[i].altitude = GPS_DEFAULT;
						gps[i].sats     = (uint8_t) GPS_DEFAULT;
						new_gps[i] = false;
					}
				}
			}
			if (new_baro[i]) {
				new_baro[i] = !handleBaroData(baro[i]);
			}
			if (new_imu[i]) {
				new_imu[i] = !handleIMUData(imu[i]);
			}
		}

		osDelay (10);
	}
}


void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	if (huart == gps_gethuart()) {
		GPS_RxCpltCallback ();
	} else if (huart == ab_gethuart()) {
		AB_RxCpltCallback();
	}
}
