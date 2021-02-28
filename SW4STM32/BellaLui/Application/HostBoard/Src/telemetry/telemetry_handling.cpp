/*
 * data_handling.c
 *
 *  Created on: 19 Apr 2018
 *      Author: Clement Nussbaumer
 *      Alexandre Devienne
 */

#include "telemetry/telemetry_handling.h"

#include "misc/data_handling.h"
#include "misc/datagram_builder.h"
#include "telemetry/simpleCRC.h"
#include "telemetry/telemetry_protocol.h"

#include <cmsis_os.h>
#include <misc/common.h>

#include <stdbool.h>


extern "C" {
	#include "debug/led.h"
	#include "debug/console.h"
	#include "can_transmission.h"
	#include "storage/sd_card.h"
}

#define TELE_TIMEMIN 200
#define GPS_TIMEMIN 200
#define STATE_TIMEMIN 200
#define PROP_DATA_TIMEMIN 100
#define AB_TIMEMIN 100
//#define TELE_RAW_TIMEMIN 100

extern osMessageQId xBeeQueueHandle;

IMU_data imu = { { 0, 0, 0 }, { 0, 0, 0 } };
BARO_data baro = { 0, 0, 0 };
uint32_t last_sensor_update = 0;
uint32_t last_motor_update = 0;
uint32_t last_propulsion_update = 0;
uint32_t last_airbrakes_update = 0;
uint32_t last_state_update = 0;
//uint32_t last_sensor_raw_update = 0;
Telemetry_Message m1;
Telemetry_Message m2;
Telemetry_Message m3;
Telemetry_Message m4;
Telemetry_Message m5;
Telemetry_Message m6;
//Telemetry_Message m7;
//Telemetry_Message m8;

Telemetry_Message event;


bool telemetrySendGPS(uint32_t timestamp, GPS_data data) {
	static uint32_t last_update = 0;
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_update > GPS_TIMEMIN) {
		m1 = createGPSDatagram(timestamp, data);
		if (osMessagePut(xBeeQueueHandle, (uint32_t) &m1, 10) != osOK) {
			vPortFree(m1.ptr); // free the datagram if we couldn't queue it
		}
		last_update = now;
		handled = true;
	}
	return handled;
}

bool telemetrySendIMU(uint32_t timestamp, IMU_data data) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	imu = data;

	if (now - last_sensor_update > TELE_TIMEMIN) {
		m2 = createTelemetryDatagram(timestamp, &imu, &baro);
		if (osMessagePut(xBeeQueueHandle, (uint32_t) &m2, 10) != osOK) {
			vPortFree(m2.ptr); // free the datagram if we couldn't queue it
		}
		last_sensor_update = now;
		handled = true;
	}
	return handled;
}

bool telemetrySendBaro(uint32_t timestamp, BARO_data data) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	baro = data;

	if (now - last_sensor_update > TELE_TIMEMIN) {
		m3 = createTelemetryDatagram(timestamp, &imu, &baro);
		if (osMessagePut(xBeeQueueHandle, (uint32_t) &m3, 10) != osOK) {
			vPortFree(m3.ptr); // free the datagram if we couldn't queue it
		}
		last_sensor_update = now;
		handled = true;
	}
	return handled;
}

bool telemetrySendAirbrakesAngle(uint32_t timestamp, float angle) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_airbrakes_update > AB_TIMEMIN) {
		m4 = createAirbrakesDatagram(timestamp, angle);

		if (osMessagePut(xBeeQueueHandle, (uint32_t) &m4, 10) != osOK) {
			vPortFree(m4.ptr); // free the datagram if we couldn't queue it
		}
		last_airbrakes_update = now;
		handled = true;
	}
	return handled;
}

bool telemetrySendState(uint32_t timestamp, bool id, float value, uint8_t av_state) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_state_update > STATE_TIMEMIN) {
		m5 = createStateDatagram(timestamp, id, value, av_state);
		if (osMessagePut(xBeeQueueHandle, (uint32_t) &m5, 10) != osOK) {
			vPortFree(m5.ptr); // free the datagram if we couldn't queue it
		}
		last_state_update = now;
		handled = true;
	}
	return handled;
}

bool telemetrySendPropulsionData(uint32_t timestamp, PropulsionData* data) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_propulsion_update > PROP_DATA_TIMEMIN) {
		m6 = createPropulsionDatagram(timestamp, data);
		if (osMessagePut(xBeeQueueHandle, (uint32_t) &m6, 10) != osOK) {
			vPortFree(m6.ptr); // free the datagram if we couldn't queue it
		}
		last_propulsion_update = now;
		handled = true;
	}
	return handled;
}

// Received Packet Handling

bool telemetryReceivePropulsionCommand(uint32_t timestamp, uint8_t* payload) {
	uint8_t command = payload[0];

	switch(command) {
	case 0x01:
		can_setFrame(0xC0FFEE, DATA_ID_START_VALVE_OPERATION, timestamp);
		break;
	case 0x02:
		can_setFrame(0xC0FFEE, DATA_ID_START_FUELING, timestamp);
		break;
	case 0x03:
		can_setFrame(0xC0FFEE, DATA_ID_STOP_FUELING, timestamp);
		break;
	case 0x04:
		can_setFrame(0xC0FFEE, DATA_ID_START_HOMING, timestamp);
		break;
	case 0x05:
		can_setFrame(0xC0FFEE, DATA_ID_ABORT, timestamp);
		break;
	default:
		rocket_boot_log("Invalid propulsion command received from GS: %d\n", command);
	}

	return 0;
}
