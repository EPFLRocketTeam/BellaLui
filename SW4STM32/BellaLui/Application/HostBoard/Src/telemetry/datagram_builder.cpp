/*
 * datagram_builder.cpp
 *
 *  Created on: 9 May 2018
 *      Author: Cl�ment Nussbaumer
 */

#include "misc/datagram_builder.h"

#include "telemetry/simpleCRC.h"
#include "telemetry/telemetry_protocol.h"

#include "Embedded/system.h"

#define MALLOC_SIZE 256 // same malloc size for all datagrams, otherwise it fragments the memory

volatile static uint32_t telemetrySeqNumber = 0;


DatagramBuilder::DatagramBuilder(uint16_t datagramPayloadSize, uint8_t datagramType, uint32_t timestamp, uint32_t seqNumber) :
		datagramSize(datagramPayloadSize + TOTAL_DATAGRAM_OVERHEAD) {
	// TODO: check datagram size < MALLOC_SIZE
	// TODO: even better => don't need size in constructor...
	datagramPtr = pvPortMalloc(MALLOC_SIZE);
	if(datagramPtr != NULL) {
		currentIdx = 0;
	} else {
		currentIdx = datagramSize; // prevents from writing anything.
		return;
	}

	//PAYLOAD TYPE
	write8(datagramType);

	//EPFL Prefix
	/*const char *EPFL_PREFIX = "EPFL";
	 write32((uint32_t) EPFL_PREFIX);*/
	write8((uint8_t) 'E');
	write8((uint8_t) 'P');
	write8((uint8_t) 'F');
	write8((uint8_t) 'L');

	write32<uint32_t>(timestamp);
	write32<uint32_t>(seqNumber);

	datagramCrc = CRC_16_GENERATOR_POLY.initialValue;
	for(int16_t i = currentIdx - 13; i < currentIdx; i++) {
		//Calculate checksum for datagram seq number and payload type fields
		datagramCrc = CalculateRemainderFromTable(*((uint8_t*) datagramPtr + i), datagramCrc);
	}
	/*
	 //CONTROL FLAG
	 write8 (CONTROL_FLAG);
	 */

}

Telemetry_Message DatagramBuilder::finalizeDatagram() {
	// TODO: HEADER_SIZE is the wrong place to start...
	for(int16_t i = (HEADER_SIZE /*+ PREAMBLE_SIZE + CONTROL_FLAG_SIZE*/); i < currentIdx; i++) {
		//Calculate checksum for datagram and payload fields
		datagramCrc = CalculateRemainderFromTable(*((uint8_t*) datagramPtr + i), datagramCrc);
	}

	datagramCrc = FinalizeCRC(datagramCrc);
	write16(datagramCrc);

	Telemetry_Message m = { .ptr = datagramPtr, .size = datagramSize };

	return m;
}

//the createXXXDatagram-Methods create the datagrams as described in the Schema (should be correct)

Telemetry_Message createTelemetryDatagram(uint32_t timestamp, IMU_data *imu_data, BARO_data *baro_data, float32_t speed, float32_t altitude) {
	DatagramBuilder builder = DatagramBuilder(SENSOR_DATAGRAM_PAYLOAD_SIZE, TELEMETRY_PACKET, timestamp, telemetrySeqNumber++);

	builder.write32<float32_t>(imu_data->acceleration.x);
	builder.write32<float32_t>(imu_data->acceleration.y);
	builder.write32<float32_t>(imu_data->acceleration.z);

	builder.write32<float32_t>(imu_data->eulerAngles.x);
	builder.write32<float32_t>(imu_data->eulerAngles.y);
	builder.write32<float32_t>(imu_data->eulerAngles.z);

	builder.write32<float32_t>(baro_data->temperature);
	builder.write32<float32_t>(baro_data->pressure);

	builder.write32<float32_t>(speed); 		// can_getSpeed()
	builder.write32<float32_t>(altitude); 	// can_getAltitude()

	return builder.finalizeDatagram();
}

Telemetry_Message createAirbrakesDatagram(uint32_t timestamp, float angle) {
	DatagramBuilder builder = DatagramBuilder(AB_DATAGRAM_PAYLOAD_SIZE, AIRBRAKES_PACKET, timestamp, telemetrySeqNumber++);

	builder.write32<float32_t>(angle); // AB_angle

	return builder.finalizeDatagram();
}

//same structure for the other createXXXDatagrams
Telemetry_Message createGPSDatagram(uint32_t timestamp, GPS_data gpsData) {
	DatagramBuilder builder = DatagramBuilder(GPS_DATAGRAM_PAYLOAD_SIZE, GPS_PACKET, timestamp, telemetrySeqNumber++);

	builder.write8(gpsData.sats);
	builder.write32<float32_t>(gpsData.hdop);
	builder.write32<float32_t>(gpsData.lat);
	builder.write32<float32_t>(gpsData.lon);
	builder.write32<int32_t>(gpsData.altitude);

	return builder.finalizeDatagram();
}

Telemetry_Message createStateDatagram(uint32_t timestamp, uint8_t id, float value, uint8_t av_state) {
	DatagramBuilder builder = DatagramBuilder(STATUS_DATAGRAM_PAYLOAD_SIZE, STATUS_PACKET, timestamp, telemetrySeqNumber++);

	builder.write8(id);
	builder.write32(value);
	builder.write8(av_state); // flight status

	return builder.finalizeDatagram();
}

Telemetry_Message createPropulsionDatagram(uint32_t timestamp, PropulsionData* data) {
	DatagramBuilder builder = DatagramBuilder(PROPULSION_DATAGRAM_PAYLOAD_SIZE, PROPULSION_DATA_PACKET, timestamp, telemetrySeqNumber++);

	builder.write16<uint16_t>(data->pressure1);
	builder.write16<uint16_t>(data->pressure2);
	builder.write16<int16_t>(data->temperature1);
	builder.write16<int16_t>(data->temperature2);
	builder.write16<int16_t>(data->temperature3);
	builder.write16<uint16_t>(data->status);
	builder.write16<int16_t>(data->motor_position);

	return builder.finalizeDatagram();
}

/*
 Telemetry_Message createOrderPacketDatagram(uint32_t time_stamp)
 {
 DatagramBuilder builder = DatagramBuilder ();
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

