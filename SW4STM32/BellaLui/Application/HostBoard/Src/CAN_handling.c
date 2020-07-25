#include <stdio.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>
#include <string.h>
#include <sys/_stdint.h>

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
#include <threads.h>
#include <cmsis_os.h>

#include <CAN_communication.h>
#include <debug/led.h>
#include <telemetry/telemetry_handling.h>
#include <airbrakes/airbrake.h>
#include <sensors/GPS_board.h>
#include <sensors/sensor_board.h>
#include <GSE_code/code.h>
#include <misc/datastructs.h>
#include <misc/Common.h>
#include <storage/sd_card.h>
#include <CAN_handling.h>
//#include <kalman/tiny_ekf.h>
#include <debug/console.h>
#include <storage/flash_logging.h>


#define BUFFER_SIZE 128

#define GPS_DEFAULT (-1.0)

IMU_data IMU_buffer[CIRC_BUFFER_SIZE];
BARO_data BARO_buffer[CIRC_BUFFER_SIZE];

float kalman_z  = 0;
float kalman_vz = 0;
float motor_pressure = 0;
GSE_state GSE = {1111,0,0,0,0};
uint8_t order = 0;
uint8_t ignition_order = 0;
uint8_t GST_code = 0;
int32_t ab_angle = 42;
uint8_t GST_code_result = 0;


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
	return telemetry_sendGPSData(data);
#elif defined(KALMAN)
	if (data.lat < 1e3) {
		return kalman_handleGPSData(data);
	}
#endif
	return false;
}

bool handleIMUData(IMU_data data) {
	IMU_buffer[(++currentImuSeqNumber) % CIRC_BUFFER_SIZE] = data;
#ifdef XBEE
	return telemetry_sendIMUData(data);
#elif defined(KALMAN)
	return kalman_handleIMUData(data);
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

	BARO_buffer[(++currentBaroSeqNumber) % CIRC_BUFFER_SIZE] = data;
	currentBaroTimestamp = HAL_GetTick();

#ifdef XBEE
	return telemetry_sendBaroData(data);
#elif defined(KALMAN)
	return kalman_handleBaroData(data);
#endif
	return false;
}

bool handleABData(int32_t new_angle) {
#ifdef XBEE
	return telemetry_sendABData();
#else
	ab_angle = can_getABangle();
#endif
	return true;
}

bool handleMotorData(IMU_data data) {
#ifdef XBEE
	//return telemetry_handleMotorPressureData(data);
#else
	//IMU_buffer[(++currentImuSeqNumber) % CIRC_BUFFER_SIZE] = data;
#endif
	return true;
}

bool handleGSEStateData(GSE_state data) {
#ifdef XBEE
	return telemetry_sendGSEStateData(data);
#endif
	return true;
}

bool handleOrderData(uint8_t order) {
#ifdef XBEE
	return telemetry_sendOrderData(order);
#endif
	return true;
}

bool handleIgnitionData(uint8_t GSE_ignition) {
#ifdef XBEE
	return telemetry_sendIgnitionData(GSE_ignition);
#endif
	return true;
}

bool handleGSTCodeData(uint8_t GST_code) {
	GST_code_result = verify_security_code(GST_code);
#ifdef XBEE
	return telemetry_sendGSTCodeData(GST_code_result);
#endif
	return true;
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

uint8_t can_getOrder() {
	return order;
}

uint8_t can_getGSTCode() {
	return GST_code;
}

uint8_t can_getIgnitionOrder() {
	return ignition_order;
}

GSE_state can_getGSEState() {
	return GSE;
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
	bool new_ab = 0;
	bool new_motor_pressure = 0;
	bool new_code = 0;
	bool new_order = 0;
	bool new_GSE_state = 0;
	bool new_GST_code = 0;
	bool new_GSE_ignition_order = 0;
	bool new_ignition_state = 0;
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
				//telemetry_handleWarningPacketData(EVENT, 0, currentState = msg.data);
#endif
				break;
			case DATA_ID_KALMAN_Z:
				kalman_z = ((float32_t) ((int32_t) msg.data))/1e3; // from mm to m
				break;
			case DATA_ID_KALMAN_VZ:
				kalman_vz = ((float32_t) ((int32_t) msg.data))/1e3; // from mm/s to m/s
				break;
			case DATA_ID_AB_INC:
				ab_angle = ((int32_t) msg.data); // keep in deg
				// new_ab = true;
				break;
			case DATA_ID_MOTOR_PRESSURE:
				motor_pressure = (float) msg.data;
				new_motor_pressure = true;
				break;
			case DATA_ID_ORDER:
				order = msg.data;
				new_order = true;
				break;
			case DATA_ID_IGNITION:
				ignition_order = msg.data;
				new_GSE_ignition_order = true;
				break;
			case DATA_ID_GSE_CODE:
				GSE.code = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_GST_CODE:
				GST_code = msg.data;
				new_GST_code = true;
				break;
			case DATA_ID_FILL_VALVE_STATE:
				GSE.fill_valve_state = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_PURGE_VALVE_STATE:
				GSE.purge_valve_state = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_HOSE_DISCONNECT_STATE:
				GSE.host_disconnect_state = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_MAIN_IGNITION_STATE:
				GSE.main_ignition_state = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_SEC_IGNITION_STATE:
				GSE.sec_ignition_state = msg.data;
				new_GSE_state = true;
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

		if (new_ab) {
			new_ab = !handleABData(ab_angle);
		} /*
		if (new_motorpressure) {
			new_motor_pressure = !handleMotorPressureData(motor_pressure);
			if (motor_pressure>XXX) {
				telemetry_handleWarningPacketData(WARNING_MOTOR_PRESSURE, motor_pressure, currentState);
			}
		}
		 */

		//Handle Rx packets, send back to GST as ACK
		if(new_order) {
			new_order = !handleOrderData(order);
		}
		if(new_GSE_ignition_order) {
			new_GSE_ignition_order = !handleIgnitionData(ignition_order);
		}
		if(new_GST_code) {
			new_GST_code = !handleGSTCodeData(GST_code);
		}
		//Handle new GSE states
		if(new_GSE_state){
			new_GSE_state = !handleGSEStateData(GSE);
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
	} else if (huart == xbee_gethuart()){
		xBee_rxCpltCallback();
	}

}
