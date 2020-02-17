/*
 * data_handling.c
 *
 *  Created on: 19 Apr 2018
 *      Author: Clement Nussbaumer
 *      Alexandre Devienne
 */

#include <misc/Common.h>
#include <misc/data_handling.h>
#include <misc/datagram_builder.h>
#include "cmsis_os.h"

#include <stdbool.h>
#include <telemetry/simpleCRC.h>
#include <telemetry/telemetry_protocol.h>

extern "C" {
	#include <CAN_communication.h>
	#include <storage/sd_card.h>
}


#define TELE_TIMEMIN 100
#define GPS_TIMEMIN 500
#define MOTOR_TIMEMIN 100
#define WARNING_TIMEMIN 100
#define AB_TIMEMIN 500
//#define TELE_RAW_TIMEMIN 100


// for import in C code
extern "C" bool telemetry_handleGPSData(GPS_data data);
extern "C" bool telemetry_handleIMUData(IMU_data data);
extern "C" bool telemetry_handleBaroData(BARO_data data);
extern "C" bool telemetry_handleWarningPacketData(bool id, uint32_t value);
extern "C" bool telemetry_handleMotorPressureData(uint32_t pressure);
extern "C" bool telemetry_handleABData(uint32_t ab_angle);

extern osMessageQId xBeeQueueHandle;

uint32_t telemetrySeqNumber = 0;

IMU_data  imu  = {{0,0,0},{0,0,0}, 0};
BARO_data baro = {0,0,0};
uint32_t last_sensor_update = 0;
uint32_t last_motor_update = 0;
uint32_t last_warning_update = 0;
uint32_t last_airbrakes_update = 0;
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

//the createXXXDatagram-Methods create the datagrams as described in the Schema (should be correct)

Telemetry_Message createTelemetryDatagram (IMU_data* imu_data, BARO_data* baro_data, uint32_t time_stamp, uint32_t telemetrySeqNumber)
{

	//here, the Datagram is created
	DatagramBuilder builder = DatagramBuilder (SENSOR_DATAGRAM_PAYLOAD_SIZE, TELEMETRY_ERT18, telemetrySeqNumber);

	// ## Beginning of datagram Payload ##
	// time stamp
	builder.write32<uint32_t> (time_stamp);

	builder.write32<float32_t> (imu_data->acceleration.x);
	builder.write32<float32_t> (imu_data->acceleration.y);
	builder.write32<float32_t> (imu_data->acceleration.z);

	builder.write32<float32_t> (imu_data->eulerAngles.x);
	builder.write32<float32_t> (imu_data->eulerAngles.y);
	builder.write32<float32_t> (imu_data->eulerAngles.z);

	builder.write32<float32_t> (baro_data->temperature);
	builder.write32<float32_t> (baro_data->pressure);

	builder.write32<float32_t> (can_getSpeed()); // pitot_press
	//baro_data->temperature = 20;


	builder.write32<float32_t> (can_getAltitude());

	return builder.finalizeDatagram ();
}

Telemetry_Message createAirbrakesDatagram (uint32_t time_stamp, uint32_t telemetrySeqNumber)
{
	DatagramBuilder builder = DatagramBuilder (AB_DATAGRAM_PAYLOAD_SIZE, AIRBRAKES, telemetrySeqNumber++);
	builder.write32<uint32_t> (time_stamp);
	builder.write32<float32_t> (can_getABangle()); // AB_angle

	return builder.finalizeDatagram ();
}

//same structure for the other createXXXDatagrams
Telemetry_Message createGPSDatagram (uint32_t seqNumber, GPS_data gpsData)
{
	DatagramBuilder builder = DatagramBuilder (GPS_DATAGRAM_PAYLOAD_SIZE, GPS, seqNumber++);

	builder.write32<uint32_t> (HAL_GetTick ());
	builder.write8 (gpsData.sats);
	builder.write32<float32_t> (gpsData.hdop);
	builder.write32<float32_t> (gpsData.lat);
	builder.write32<float32_t> (gpsData.lon);
	builder.write32<int32_t> (gpsData.altitude);

	return builder.finalizeDatagram ();
}

Telemetry_Message createMotorPressurePacketDatagram(uint32_t time_stamp, float32_t pressure, uint32_t seqNumber)
{
	DatagramBuilder builder = DatagramBuilder (MOTORPRESSURE_DATAGRAM_PAYLOAD_SIZE, MOTOR, seqNumber++);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<float32_t> (pressure);

	return builder.finalizeDatagram();
}
//new
Telemetry_Message createWarningPacketDatagram(uint32_t time_stamp, bool id, uint32_t value, uint32_t seqNumber)
{
	DatagramBuilder builder = DatagramBuilder (WARNING_DATAGRAM_PAYLOAD_SIZE, EVENT, seqNumber++);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<bool> (id);
	builder.write32<uint32_t> (value);
	builder.write32<float32_t> (can_getState()); // flight status

	return builder.finalizeDatagram();
}

/*
Telemetry_Message createOrderPacketDatagram(uint32_t time_stamp)
{
	DatagramBuilder builder = DatagramBuilder ();
}
*/

/*
Telemetry_Message createTelemetryRawDatagram(uint32_t time_stamp, float32_t euler, float32_t accelerometer, float32_t temp, float32_t pressure, uint32_t seqNumber)
{
	DatagramBuilder builder = DatagramBuilder (TELEMETRY_RAW_DATAGRAM_PAYLOAD_SIZE,time_stamp);

	builder.write32<float32_t> (euler);
	builder.write32<float32_t> (accelerometer);
	builder.write32<float32_t> (temp);
	builder.write32<float32_t> (pressure);

	return builder.finalizeDatagram();
}
*/

/*
Telemetry_Message createEventDatagram(uint32_t time_stamp, uint32_t state)
{
	DatagramBuilder builder = DatagramBuilder (EVENT_DATAGRAM_PAYLOAD_SIZE,time_stamp);

	builder.write32<uint32_t> (state);

	return builder.finalizeDatagram();
}
*/

// New Packets
/*
	Send:
	telemetry-raw
	telemetry-filtered (after kalman)
	motorPressure
	eventState (FSM)
	warning Packet

	Receive:
	order Packet (fill tank, abort)
	ignition (ouvrir la vanne, etc) (go)
	eventState(FSM)
*/

//New methods to implement :
//createOrderPackerDatagram
//createIgnitionDatagram


bool telemetry_handleGPSData(GPS_data data) {
	static uint32_t last_update = 0;
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_update > GPS_TIMEMIN) {
		m1 = createGPSDatagram (telemetrySeqNumber++, data);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m1, 10) != osOK) {
			vPortFree(m1.ptr); // free the datagram if we couldn't queue it
		}
		last_update = now;
		handled = true;
	}
	return handled;
}

bool telemetry_handleIMUData(IMU_data data) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	imu = data;

	if (now - last_sensor_update > TELE_TIMEMIN) {
		m2 = createTelemetryDatagram (&imu, &baro, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m2, 10) != osOK) {
			vPortFree(m2.ptr); // free the datagram if we couldn't queue it
		}
		last_sensor_update = now;
		handled = true;
	}
	return handled;
}

bool telemetry_handleBaroData(BARO_data data) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	baro = data;

	if (now - last_sensor_update > TELE_TIMEMIN) {
		m3 = createTelemetryDatagram (&imu, &baro, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m3, 10) != osOK) {
			vPortFree(m3.ptr); // free the datagram if we couldn't queue it
		}
		last_sensor_update = now;
		handled = true;
	}
	return handled;
}

bool telemetry_handleMotorPressureData(uint32_t pressure)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_motor_update > MOTOR_TIMEMIN) {
		m4 = createMotorPressurePacketDatagram (pressure, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m4, 10) != osOK) {
			vPortFree(m4.ptr); // free the datagram if we couldn't queue it
		}
		last_motor_update = now;
		handled = true;
	}
	return handled;
}

bool telemetry_handleWarningPacketData(bool id, uint32_t value)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_warning_update > WARNING_TIMEMIN) {
		m5 = createWarningPacketDatagram (now, id, value, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m5, 10) != osOK) {
			vPortFree(m5.ptr); // free the datagram if we couldn't queue it
		}
		last_warning_update = now;
		handled = true;
	}
	return handled;
}

bool telemetry_handleABData(uint32_t ab_angle) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_airbrakes_update > AB_TIMEMIN) {
		m6 = createTelemetryDatagram (&imu, &baro, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m6, 10) != osOK) {
			vPortFree(m6.ptr); // free the datagram if we couldn't queue it
		}
		last_airbrakes_update = now;
		handled = true;
	}
	return handled;
}

/*
bool telemetry_handleTelemetryRaw(float32_t euler, float32_t accelerometer, float32_t temp, float32_t pressure)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_sensor_raw_update > TELE_RAW_TIMEMIN) {
		m7 = createTelemetryRawDatagram(now, euler, accelerometer, temp, pressure, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m7, 10) != osOK) {
			vPortFree(m7.ptr); // free the datagram if we couldn't queue it
		}
		last_sensor_raw_update = now;
		handled = true;
	}
	return handled;
}
*/

/*
bool telemetry_handleEventData(uint32_t state)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_event_update > EVENT_TIMEMIN) {
		m8 = createEventDatagram (now, state, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m8, 10) != osOK) {
			vPortFree(m8.ptr); // free the datagram if we couldn't queue it
		}
		last_event_update = now;
		handled = true;
	}
	return handled;
}
*/

