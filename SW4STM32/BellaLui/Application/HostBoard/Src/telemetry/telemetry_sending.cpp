/*
 * telemetry_sending.cpp
 *
 *  Created on: 19 Apr 2018
 *      Author: Clement Nussbaumer
 *      Alexandre Devienne
 *      ofacklam
 */

#include "telemetry/telemetry_sending.h"

#include "telemetry/datagram_builder.h"
#include "telemetry/telemetry_protocol.h"

#include <Embedded/system.h>
#include <Embedded/can.h>

#include <stdbool.h>


#define TELE_TIMEMIN 200
#define GPS_TIMEMIN 200
#define STATE_TIMEMIN 200
#define PROP_DATA_TIMEMIN 100
#define AB_TIMEMIN 100
//#define TELE_RAW_TIMEMIN 100

AbstractMessageQueue<Telemetry_Message> *msgQueue = nullptr;

IMU_data imu = { { 0, 0, 0 }, { 0, 0, 0 } };
BARO_data baro = { 0, 0, 0 };
uint32_t last_gps_update = 0;
uint32_t last_sensor_update = 0;
uint32_t last_motor_update = 0;
uint32_t last_propulsion_update = 0;
uint32_t last_airbrakes_update = 0;
uint32_t last_state_update = 0;
//uint32_t last_sensor_raw_update = 0;


void registerSendQueue(AbstractMessageQueue<Telemetry_Message> *queue) {
	msgQueue = queue;
}


bool telemetrySendGPS(uint32_t timestamp, GPS_data data) {
	uint32_t now = HAL_GetTick();

	if (now - last_gps_update <= GPS_TIMEMIN || nullptr == msgQueue)
		return false;

	Telemetry_Message *msg = createGPSDatagram(timestamp, data);
	if (!msgQueue->push(msg, 10)) {
		vPortFree(msg); // free the datagram if we couldn't queue it
	}
	last_gps_update = now;
	return true;
}

bool telemetrySendIMU(uint32_t timestamp, IMU_data data) {
	uint32_t now = HAL_GetTick();
	imu = data;

	if (now - last_sensor_update <= TELE_TIMEMIN || nullptr == msgQueue)
		return false;

	Telemetry_Message *msg = createTelemetryDatagram(timestamp, &imu, &baro, can_getSpeed(), can_getAltitude());
	if (!msgQueue->push(msg, 10)) {
		vPortFree(msg); // free the datagram if we couldn't queue it
	}
	last_sensor_update = now;
	return true;
}

bool telemetrySendBaro(uint32_t timestamp, BARO_data data) {
	uint32_t now = HAL_GetTick();
	baro = data;

	if (now - last_sensor_update <= TELE_TIMEMIN || nullptr == msgQueue)
		return false;

	Telemetry_Message *msg = createTelemetryDatagram(timestamp, &imu, &baro, can_getSpeed(), can_getAltitude());
	if (!msgQueue->push(msg, 10)) {
		vPortFree(msg); // free the datagram if we couldn't queue it
	}
	last_sensor_update = now;
	return true;
}

bool telemetrySendAirbrakesAngle(uint32_t timestamp, float angle) {
	uint32_t now = HAL_GetTick();

	if (now - last_airbrakes_update <= AB_TIMEMIN || nullptr == msgQueue)
		return false;

	Telemetry_Message *msg = createAirbrakesDatagram(timestamp, angle);
	if (!msgQueue->push(msg, 10)) {
		vPortFree(msg); // free the datagram if we couldn't queue it
	}
	last_airbrakes_update = now;
	return true;
}

bool telemetrySendState(uint32_t timestamp, bool id, float value, uint8_t av_state) {
	uint32_t now = HAL_GetTick();

	if (now - last_state_update <= STATE_TIMEMIN || nullptr == msgQueue)
		return false;

	Telemetry_Message *msg = createStateDatagram(timestamp, id, value, av_state);
	if (!msgQueue->push(msg, 10)) {
		vPortFree(msg); // free the datagram if we couldn't queue it
	}
	last_state_update = now;
	return true;
}

bool telemetrySendPropulsionData(uint32_t timestamp, PropulsionData* data) {
	uint32_t now = HAL_GetTick();

	if (now - last_propulsion_update <= PROP_DATA_TIMEMIN || nullptr == msgQueue)
		return false;

	Telemetry_Message *msg = createPropulsionDatagram(timestamp, data);
	if (!msgQueue->push(msg, 10)) {
		vPortFree(msg); // free the datagram if we couldn't queue it
	}
	last_propulsion_update = now;
	return true;
}
