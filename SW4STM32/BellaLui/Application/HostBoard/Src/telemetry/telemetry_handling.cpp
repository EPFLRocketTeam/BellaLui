/*
 * data_handling.c
 *
 *  Created on: 19 Apr 2018
 *      Author: Clement Nussbaumer
 *      Alexandre Devienne
 */

#include "../../../HostBoard/Inc/debug/led.h"
#include <misc/Common.h>
#include <misc/data_handling.h>
#include <misc/datagram_builder.h>
#include "cmsis_os.h"

#include <stdbool.h>
#include <telemetry/simpleCRC.h>
#include <telemetry/telemetry_protocol.h>
#include <GSE/code.h>

extern "C" {
	#include <can_transmission.h>
	#include <storage/sd_card.h>
	#include <debug/console.h>
}

#define TELE_TIMEMIN 200
#define GPS_TIMEMIN 200
#define STATE_TIMEMIN 200
#define PROP_DATA_TIMEMIN 100
#define AB_TIMEMIN 100
#define GSE_STATE_TIMEMIN 20
#define ORDER_TIMEMIN 20
#define GSE_IGNITION_TIMEMIN 20
#define ECHO_TIMEMIN 20
//#define TELE_RAW_TIMEMIN 100

volatile static uint32_t Packet_Number = 0;

// for import in C code
extern "C" bool telemetry_sendGPSData(GPS_data data);
extern "C" bool telemetry_sendIMUData(IMU_data data);
extern "C" bool telemetry_sendBaroData(BARO_data data);
extern "C" bool telemetry_sendWarningPacketData(bool id, float value, uint8_t av_state);
extern "C" bool telemetry_sendMotorPressureData(uint32_t pressure);
extern "C" bool telemetry_sendABData();
extern "C" bool telemetry_sendGSEStateData(GSE_state data);
extern "C" bool telemetry_sendOrderData(uint8_t order);
extern "C" bool telemetry_sendIgnitionData(uint8_t GSE_ignition);
extern "C" bool telemetry_sendEcho(uint8_t GST_code_result);

extern "C" bool telemetry_receiveOrderPacket(uint32_t time_stamp, uint8_t* payload);
extern "C" bool telemetry_receiveIgnitionPacket(uint32_t time_stamp, uint8_t* payload);

extern osMessageQId xBeeQueueHandle;

uint32_t telemetrySeqNumber = 0;
IMU_data imu = {{0,0,0},{0,0,0}, 0};
BARO_data baro = {0,0,0};
uint32_t last_sensor_update = 0;
uint32_t last_motor_update = 0;
uint32_t last_warning_update = 0;
uint32_t last_airbrakes_update = 0;
uint32_t last_state_update = 0;
uint8_t current_GSE_order = 0;

GSE_state GSE_states = {0,0,0,0,0,1111,0,0,0,0,0,0,0,0};
uint8_t sec_GST_code = 0;
uint32_t last_GSE_state_update = 0;
uint32_t last_order_update = 0;
uint32_t last_GSE_ignition_update = 0;
uint32_t last_echo = 0;
//uint32_t last_sensor_raw_update = 0;
Telemetry_Message m1;
Telemetry_Message m2;
Telemetry_Message m3;
Telemetry_Message m4;
Telemetry_Message m5;
Telemetry_Message m6;
Telemetry_Message m7;
Telemetry_Message m8;
Telemetry_Message m9;
Telemetry_Message m10;


Telemetry_Message event;

//the createXXXDatagram-Methods create the datagrams as described in the Schema (should be correct)

Telemetry_Message createTelemetryDatagram (IMU_data* imu_data, BARO_data* baro_data, uint32_t time_stamp, uint32_t seqNumber)
{

	//here, the Datagram is created
	DatagramBuilder builder = DatagramBuilder (SENSOR_DATAGRAM_PAYLOAD_SIZE, TELEMETRY_PACKET, time_stamp, seqNumber);

	// ## Beginning of datagram Payload ##
	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
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

Telemetry_Message createAirbrakesDatagram (uint32_t time_stamp, uint32_t seqNumber)
{
	DatagramBuilder builder = DatagramBuilder (AB_DATAGRAM_PAYLOAD_SIZE, AIRBRAKES_PACKET,time_stamp, seqNumber);
	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
	builder.write32<float32_t> (can_getABangle()); // AB_angle

	return builder.finalizeDatagram ();
}

//same structure for the other createXXXDatagrams
Telemetry_Message createGPSDatagram (uint32_t time_stamp, GPS_data gpsData, uint32_t seqNumber)
{
	DatagramBuilder builder = DatagramBuilder (GPS_DATAGRAM_PAYLOAD_SIZE, GPS_PACKET, time_stamp, seqNumber);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
	builder.write8 (gpsData.sats);
	builder.write32<float32_t> (gpsData.hdop);
	builder.write32<float32_t> (gpsData.lat);
	builder.write32<float32_t> (gpsData.lon);
	builder.write32<int32_t> (gpsData.altitude);

	return builder.finalizeDatagram ();
}

Telemetry_Message createMotorPressurePacketDatagram(uint32_t time_stamp, float32_t pressure, uint32_t seqNumber)
{
	DatagramBuilder builder = DatagramBuilder (MOTORPRESSURE_DATAGRAM_PAYLOAD_SIZE, MOTOR_PACKET, time_stamp, seqNumber);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
	builder.write32<float32_t> (pressure);

	return builder.finalizeDatagram();
}
//new


Telemetry_Message createOrderDatagram(uint8_t order, uint32_t time_stamp, uint32_t seqNumber)
{
	DatagramBuilder builder = DatagramBuilder (ORDER_DATAGRAM_PAYLOAD_SIZE, ORDER_PACKET,time_stamp, seqNumber);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
	builder.write8 (order);
	return builder.finalizeDatagram();

}

Telemetry_Message createIgnitionDatagram(uint8_t GSE_ignition, uint32_t time_stamp, uint32_t seqNumber)
{
	//GST_code_result = verify_security_code(GST_code);
	uint8_t GST_code_result = 0;

	DatagramBuilder builder = DatagramBuilder (GSE_IGNITION_DATAGRAM_PAYLOAD_SIZE, IGNITION_PACKET, time_stamp, seqNumber);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
	builder.write8 (GSE_ignition);
	builder.write8(GST_code_result);
	return builder.finalizeDatagram();

}

Telemetry_Message createEchoDatagram(uint32_t time_stamp, uint32_t seqNumber)
{

	DatagramBuilder builder = DatagramBuilder (GSE_IGNITION_DATAGRAM_PAYLOAD_SIZE, ECHO_PACKET,time_stamp, seqNumber);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
	builder.write8(0xCA);
	return builder.finalizeDatagram();

}
Telemetry_Message createGSEStateDatagram(GSE_state* GSE, uint32_t time_stamp, uint32_t seqNumber) {
	DatagramBuilder builder = DatagramBuilder(GSE_STATE_DATAGRAM_PAYLOAD_SIZE, GSE_STATE_PACKET,time_stamp, seqNumber);

	builder.write8 (GSE->fill_valve_state);
	builder.write8 (GSE->purge_valve_state);
	builder.write8 (GSE->main_ignition_state);
	builder.write8 (GSE->sec_ignition_state);
	builder.write8 (GSE->hose_disconnect_state);
	builder.write32<float32_t>(GSE->battery_level);
	builder.write32<float32_t>(GSE->hose_pressure);
	builder.write32<float32_t>(GSE->hose_temperature);
	builder.write32<float32_t>(GSE->tank_temperature);
	builder.write32<float32_t>(GSE->rocket_weight);
	builder.write32<float32_t>(GSE->ignition1_current);
	builder.write32<float32_t>(GSE->ignition2_current);
	builder.write32<float32_t>(GSE->wind_speed);

	return builder.finalizeDatagram();
}

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
	ignition  (go)
*/

//New methods to implement :
//createOrderPackerDatagram
//createIgnitionDatagram


bool telemetry_sendGPSData(GPS_data data) {
	static uint32_t last_update = 0;
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_update > GPS_TIMEMIN) {
		m1 = createGPSDatagram (now, data, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m1, 10) != osOK) {
			vPortFree(m1.ptr); // free the datagram if we couldn't queue it
		}
		last_update = now;
		handled = true;
	}
	return handled;
}

bool telemetry_sendIMUData(IMU_data data) {
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

bool telemetry_sendBaroData(BARO_data data) {
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

bool telemetry_sendGSEStateData(GSE_state data)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_GSE_state_update > GSE_STATE_TIMEMIN) {
		m7 = createGSEStateDatagram(&data, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m7, 10) != osOK) {
			vPortFree(m7.ptr); // free the datagram if we couldn't queue it
		}
		last_GSE_state_update = now;
		handled = true;

	}
	return handled;
}

bool telemetry_sendOrderData(uint8_t order)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_order_update > ORDER_TIMEMIN) {
		m8 = createOrderDatagram(order, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m8, 10) != osOK) {
			vPortFree(m8.ptr); // free the datagram if we couldn't queue it
		}
		last_order_update = now;
		handled = true;

	}
	return handled;
}

bool telemetry_sendIgnitionData(uint8_t GSE_ignition)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_GSE_ignition_update > GSE_IGNITION_TIMEMIN) {
		m9 = createIgnitionDatagram(GSE_ignition, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m9, 10) != osOK) {
			vPortFree(m9.ptr); // free the datagram if we couldn't queue it
		}
		last_GSE_ignition_update = now;
		handled = true;

	}
	return handled;
}

bool telemetry_sendEcho()
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_echo > ECHO_TIMEMIN) {
		m10 = createEchoDatagram(now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m10, 10) != osOK) {
			vPortFree(m10.ptr); // free the datagram if we couldn't queue it
		}
		last_echo = now;
		handled = true;

	}
	return handled;
}

// Received Packet Handling


// Received Packet Handling

bool telemetry_receiveOrderPacket(uint32_t ts, uint8_t* payload) {

	switch (payload[0])
	{
		case OPEN_FILL_VALVE:
		{
			current_GSE_order = OPEN_FILL_VALVE;
			break;
		}
		case CLOSE_FILL_VALVE:
		{
			current_GSE_order = CLOSE_FILL_VALVE;
			break;
		}
		case OPEN_PURGE_VALVE:
		{
			current_GSE_order = OPEN_PURGE_VALVE;
			break;
		}
		case CLOSE_PURGE_VALVE:
		{
			current_GSE_order = CLOSE_PURGE_VALVE;
			break;
		}
		case OPEN_FILL_VALVE_BACKUP:
		{
			current_GSE_order = OPEN_FILL_VALVE_BACKUP;
			break;
		}
		case CLOSE_FILL_VALVE_BACKUP:
		{
			current_GSE_order = CLOSE_FILL_VALVE_BACKUP;
			break;
		}
		case OPEN_PURGE_VALVE_BACKUP:
		{
			current_GSE_order = OPEN_PURGE_VALVE_BACKUP;
			break;
		}
		case CLOSE_PURGE_VALVE_BACKUP:
		{
			current_GSE_order = CLOSE_PURGE_VALVE_BACKUP;
			break;
		}
		case DISCONNECT_HOSE:
		{
			current_GSE_order = DISCONNECT_HOSE;
			break;
		}
	}
	can_setFrame((uint32_t) current_GSE_order, DATA_ID_ORDER , ts);
	return 0;
}

bool telemetry_receiveIgnitionPacket(uint32_t ts, uint8_t* payload) {
	if(payload[0] == MAIN_IGNITION_ON) {
		can_setFrame((uint32_t) MAIN_IGNITION_ON, DATA_ID_IGNITION, ts);
	}
	else if(payload[0] == MAIN_IGNITION_OFF) {
			can_setFrame((uint32_t) MAIN_IGNITION_OFF, DATA_ID_IGNITION, ts);
	}
	else if(payload[0] == SECONDARY_IGNITION_ON) {
		can_setFrame((uint32_t) SECONDARY_IGNITION_ON, DATA_ID_IGNITION, ts);
	}
	else if(payload[0] == SECONDARY_IGNITION_OFF) {
		can_setFrame((uint32_t) SECONDARY_IGNITION_OFF, DATA_ID_IGNITION, ts);
	}
	can_setFrame((uint32_t) payload[1], DATA_ID_GST_CODE, ts);
	return 0;
}


