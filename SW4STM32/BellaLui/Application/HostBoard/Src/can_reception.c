
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

#include "can_transmission.h"
#include "debug/profiler.h"
#include "debug/led.h"
#include "debug/monitor.h"
#include "telemetry/telemetry_sending.h"
#include "airbrakes/airbrake.h"
#include "misc/datastructs.h"
#include <misc/common.h>
#include "storage/flash_logging.h"
#include "debug/console.h"
#include "storage/flash_logging.h"
#include "debug/terminal.h"
#include <misc/rocket_constants.h>


#define BUFFER_SIZE 128
#define OUTPUT_SHELL_BUFFER_SIZE 256
#define SHELL_MIN_FLUSH_TIME 100
#define GPS_DEFAULT (-1.0)
#define MAX_MOTOR_PRESSURE 10000.0f // TO BE CONFIGURED

IMU_data IMU_buffer[CIRC_BUFFER_SIZE];
BARO_data BARO_buffer[CIRC_BUFFER_SIZE];

float kalman_z  = 0;
float kalman_vz = 0;
int32_t ab_angle = 42;


float altitudeFromPressure(float pressure_hPa) {
	return 44330 * (1.0 - pow (pressure_hPa / ADJUSTED_SEA_LEVEL_PRESSURE, 0.1903));
}

// wrapper to avoid fatal crashes when implementing redundancy
int board2Idx(uint32_t board) {
	if (board < MAX_BOARD_ID) {
		return board;
	}  else { // invalid board ID
		// to avoid fatal crash return a default value
		return MAX_BOARD_ID;
	}
}

bool handleIMUData(uint32_t timestamp, IMU_data data) {
	IMU_buffer[(++currentImuSeqNumber) % CIRC_BUFFER_SIZE] = data;
	#ifdef XBEE
	return telemetrySendIMU(timestamp, data);
	#elif defined(KALMAN)
	return kalmanProcessIMU(data);
	#endif
	return true;
}

bool handleBaroData(uint32_t timestamp, BARO_data data) {
	data.altitude = altitudeFromPressure(data.pressure);

	#ifdef CERNIER_LEGACY_DATA
	data.base_pressure = 938.86;
	#endif

	if (data.base_pressure > 0) { // Warning: terrible HOTFIX, I know
		data.base_altitude = altitudeFromPressure(data.base_pressure);
	}

	BARO_buffer[(++currentBaroSeqNumber) % CIRC_BUFFER_SIZE] = data;
	currentBaroTimestamp = HAL_GetTick();

	#ifdef XBEE
	return telemetrySendBaro(timestamp, data);
	#elif defined(KALMAN)
	return kalmanProcessBaro(data);
	#endif
	return false;
}

bool handleABData(uint32_t timestamp, int32_t new_angle) {
	#ifdef XBEE
	return telemetrySendAirbrakesAngle(timestamp, new_angle);
	#else
	ab_angle = new_angle;
	#endif

	return true;
}

bool handleStateUpdate(uint32_t timestamp, uint8_t state) {
	if(state == STATE_LIFTOFF) {
    	start_logging();
    	liftoff_time = timestamp;
	} else if(state == STATE_TOUCHDOWN) {
		stop_logging();
        on_dump_request();
	}

	current_state = state; // Update the system state

	return true;
}

bool handlePropulsionData(uint32_t timestamp, PropulsionData* data) {
	if(enter_monitor(PROPULSION_MONITOR)) {
		rocket_log(" Status: %x\x1b[K\n", data->status);
		rocket_log(" Temperature 1: %d\x1b[K\n", 100 * (int32_t) data->temperature1);
		rocket_log(" Temperature 2: %d [m°C]\x1b[K\n", 100 * (int32_t) data->temperature2);
		rocket_log(" Temperature 3: %d [m°C]\x1b[K\n", 100 * (int32_t) data->temperature3);
		rocket_log(" Pressure 1: %d [mBar]\x1b[K\n", (int32_t) data->pressure1);
		rocket_log(" Pressure 2: %d [mBar]\x1b[K\n", (int32_t) data->pressure1);
		rocket_log(" Motor position: %d [mdeg]\x1b[K\n", 100 * (int32_t) data->motor_position);

		exit_monitor(PROPULSION_MONITOR);
	}

	#ifdef XBEE
	telemetrySendPropulsionData(timestamp, data);
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
	return current_state;
}

int32_t can_getABangle() {
	return ab_angle;
}

/*
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
}*/

void TK_can_reader() {
	// init
	CAN_msg msg;

	IMU_data  imu [MAX_BOARD_NUMBER] = {0};
	BARO_data baro[MAX_BOARD_NUMBER] = {0};
	PropulsionData prop_data;
	uint8_t state = 0;

	bool new_baro[MAX_BOARD_NUMBER] = {0};
	bool new_imu [MAX_BOARD_NUMBER] = {0};
	bool new_ab = 0;
	bool new_prop_data = 0;
	bool new_state = 0;
	int idx = 0;
	uint32_t shell_command;
	uint32_t shell_payload;

	osDelay (500); // Wait for the other threads to be ready

	while(true) {
		start_profiler(1);

		while (can_msgPending()) { // check if new data
			msg = can_readBuffer();

			/*if((int32_t) (HAL_GetTick() - msg.timestamp) > 100000) {
				rocket_log("CAN RX Error %d@%d vs %d\n", msg.id, msg.timestamp, HAL_GetTick());
				continue;
			}*/

			if(is_verbose()) {
				rocket_log("----- CAN RX frame begins -----\r\n");
				rocket_log("Frame Source: %d\r\n", (uint32_t) msg.id_CAN);
				rocket_log("Frame ID: %d\r\n", (uint32_t) msg.id);
				rocket_log("Frame Timestamp: %d\r\n", (uint32_t) msg.timestamp);
				rocket_log("Frame Data: %d\r\n", (uint32_t) msg.data);
				rocket_log("----- CAN RX frame ends -----\r\n");
			}


			// add to SD card
#ifdef SDCARD
				sendSDcard(msg);
#endif

			idx = board2Idx(msg.id_CAN);

			switch(msg.id) {
			case DATA_ID_PRESSURE:
				baro[idx].pressure = ((float32_t) ((int32_t) msg.data)) / 10000; // convert from cPa to hPa
				new_baro[idx] = true; // only update when we get the pressure
				break;
			case DATA_ID_TEMPERATURE:
				baro[idx].temperature = ((float32_t) ((int32_t) msg.data)) / 100; // from to cDegC in DegC
				break;
			case DATA_ID_CALIB_PRESSURE:
				baro[idx].base_pressure = ((float32_t) ((int32_t) msg.data)) / 10000; // from cPa to hPa
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
			case DATA_ID_STATE:
				if(msg.data > state && msg.data < NUM_STATES) {
					new_state = true;
					state = msg.data;
				}

				#ifndef ROCKET_FSM // to avoid self loop on board with FSM
					telemetrySendState(msg.timestamp, EVENT, 0, current_state);
				#endif

				break;
			case DATA_ID_KALMAN_STATE:
				break;
			case DATA_ID_KALMAN_Z:
				kalman_z = ((float32_t) ((int32_t) msg.data))/1e3; // from mm to m
				break;
			case DATA_ID_KALMAN_VZ:
				kalman_vz = ((float32_t) ((int32_t) msg.data))/1e3; // from mm/s to m/s
				break;
			case DATA_ID_AB_STATE:
				break;
			case DATA_ID_AB_AIRSPEED:
				break;
			case DATA_ID_AB_ALT:
				break;
			case DATA_ID_AB_INC:
				ab_angle = ((int32_t) msg.data); // keep in deg
				// new_ab = true;
				break;
			case DATA_ID_PROP_PRESSURE1:
				prop_data.pressure1 = msg.data & 0xFFFF;
				new_prop_data = true;
				break;
			case DATA_ID_PROP_PRESSURE2:
				prop_data.pressure2 = msg.data & 0xFFFF;
				new_prop_data = true;
				break;
			case DATA_ID_PROP_TEMPERATURE1:
				prop_data.temperature1 = (int16_t) (msg.data & 0xFFFF);
				new_prop_data = true;
				break;
			case DATA_ID_PROP_TEMPERATURE2:
				prop_data.temperature2 = (int16_t) (msg.data & 0xFFFF);
				new_prop_data = true;
				break;
			case DATA_ID_PROP_TEMPERATURE3:
				prop_data.temperature3 = (int16_t) (msg.data & 0xFFFF);
				new_prop_data = true;
				break;
			case DATA_ID_PROP_STATUS:
				prop_data.status = msg.data & 0xFFFF;
				new_prop_data = true;
				break;
			case DATA_ID_PROP_MOTOR_POSITION:
				prop_data.motor_position = (int16_t) (msg.data & 0xFFFF);
				new_prop_data = true;
				break;
			case DATA_ID_SHELL_CONTROL:
				shell_command = msg.data & 0xFF000000;
				shell_payload = msg.data & 0x00FFFFFF;

				if(shell_command == SHELL_BRIDGE_CREATE) {
					shell_bridge(shell_payload & 0xF);
					can_setFrame(SHELL_ACK, DATA_ID_SHELL_CONTROL, HAL_GetTick());
					rocket_log("\n\nBellaLui Terminal for board %u\n\n", get_board_id());
				} else if(shell_command == SHELL_BRIDGE_DESTROY) {
					shell_bridge(-1);
				} else if(shell_command == SHELL_ACK) {
					rocket_direct_transmit((uint8_t*) "> Connected to remote shell\n", 28);
				} else if(shell_command == SHELL_ERR) {
					rocket_direct_transmit((uint8_t*) "> Failed to connect to remote shell\n", 36);
				}

				break;
			case DATA_ID_SHELL_INPUT: // Little-Endian
				shell_receive_byte(((char*) &msg.data)[0], -1);
				shell_receive_byte(((char*) &msg.data)[1], -1);
				shell_receive_byte(((char*) &msg.data)[2], -1);
				shell_receive_byte(((char*) &msg.data)[3], -1);
				break;
			case DATA_ID_SHELL_OUTPUT:
				rocket_direct_transmit((uint8_t*) &msg.data, 4);
				break;
			default:
				rocket_log("Unhandled can frame ID %d\n", msg.id);
			}
		}

		// check if new/non-handled full sensor packets are present
		for (int i=0; i < MAX_BOARD_NUMBER ; i++) {
			if (new_baro[i]) {
				if(baro[i].base_pressure == 0) {
					baro[i].base_pressure = baro[i].pressure; // Terrible HOTFIX, I know.
				}

				new_baro[i] = !handleBaroData(msg.timestamp, baro[i]);
			}
			if (new_imu[i]) {
				new_imu[i] = !handleIMUData(msg.timestamp, imu[i]);
			}
		}

		if (new_ab) {
			new_ab = !handleABData(msg.timestamp, ab_angle);
		}

		if(new_state) {
			new_state = !handleStateUpdate(msg.timestamp, state);
		}

		if (new_prop_data) {
			new_prop_data = !handlePropulsionData(msg.timestamp, &prop_data);
		}

		end_profiler();

		osDelay(10);
	}
}

/*
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	if (huart == gps_gethuart()) {
		GPS_RxCpltCallback ();
	} else if (huart == ab_gethuart()) {
		AB_RxCpltCallback();
	}
}*/
