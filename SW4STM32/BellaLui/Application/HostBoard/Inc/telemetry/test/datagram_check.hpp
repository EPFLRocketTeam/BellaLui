/*
 * datagram_check.h
 *
 *  Created on: 26 Feb 2021
 *      Author: Oliver
 */

#ifndef TELEMETRY_TEST_DATAGRAM_CHECK
#define TELEMETRY_TEST_DATAGRAM_CHECK

#include <gtest/gtest.h>
#include "telemetry/datagram_builder.h"
#include "telemetry/simpleCRC.h"
#include "telemetry/telemetry_protocol.h"

/**
 * Generic helper functions (checking values / headers / footers)
 */

template<typename T>
void checkVal8(uint8_t *ptr, T expected, std::string msg) {
	EXPECT_EQ(*ptr, expected) << msg;
}

inline void checkVals(uint8_t *ptr, uint8_t *expected, int size, std::string msg) {
	for(int i = 0; i < size; i++) {
		checkVal8(ptr + i, expected[i], msg + " (index " + std::to_string(i) + ")");
	}
}

template<typename T>
void checkVal16(uint8_t *ptr, T expected, std::string msg) {
	uint16_t castVal = *((uint16_t*) &expected);
	uint16_t swappedVal = bswap16(castVal);
	uint8_t *arrByte = (uint8_t*) &swappedVal;
	checkVals(ptr, arrByte, 2, msg);
}

template<typename T>
void checkVal32(uint8_t *ptr, T expected, std::string msg) {
	uint32_t castVal = *((uint32_t*) &expected);
	uint32_t swappedVal = bswap32(castVal);
	uint8_t *arrByte = (uint8_t*) &swappedVal;
	checkVals(ptr, arrByte, 4, msg);
}


inline void checkHeader(uint8_t *ptr, uint8_t id, uint32_t ts, uint32_t seq) {
	// Datagram ID
	checkVal8(ptr, id, "Datagram ID mismatch");

	// EPFL prefix
	uint8_t prefix[] = "EPFL";
	checkVals(ptr + 1, prefix, 4, "EPFL prefix mismatch");

	// Timestamp
	checkVal32(ptr + 5, ts, "Timestamp mismatch");

	// Sequence number
	checkVal32(ptr + 9, seq, "Sequence number mismatch");
}

inline void checkCRC(uint8_t *ptr, int size) {
	uint16_t crc = CRC_16_GENERATOR_POLY.initialValue;

	for(int i = 0; i < size; i++) {
		crc = CalculateRemainderFromTable(ptr[i], crc);
	}
	crc = FinalizeCRC(crc);

	checkVal16(ptr + size, crc, "CRC mismatch");
}

/**
 * Specific functions for verifying each type of datagram
 */

inline void checkTelemetryDatagram(Telemetry_Message *msg, uint32_t ts, uint32_t seq, IMU_data imu, BARO_data baro, float speed, float altitude) {
	int size = 40; //bytes

	ASSERT_EQ(msg->size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg->buf, TELEMETRY_PACKET, ts, seq);

	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 0, imu.acceleration.x, "Acceleration X incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 4, imu.acceleration.y, "Acceleration Y incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 8, imu.acceleration.z, "Acceleration Z incorrect");

	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 12, imu.eulerAngles.x, "Euler X incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 16, imu.eulerAngles.y, "Euler Y incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 20, imu.eulerAngles.z, "Euler Z incorrect");

	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 24, baro.temperature, "Temperature incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 28, baro.pressure, "Pressure incorrect");

	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 32, speed, "Speed incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 36, altitude, "Altitude incorrect");

	if(ACTIVATE_DATAGRAM_CHECKSUM)
		checkCRC((uint8_t*) msg->buf, msg->size - CHECKSUM_SIZE);
}

inline void checkAirbrakesDatagram(Telemetry_Message *msg, uint32_t ts, uint32_t seq, float angle) {
	int size = 4; //bytes

	ASSERT_EQ(msg->size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg->buf, AIRBRAKES_PACKET, ts, seq);

	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER, angle, "Angle incorrect");

	if(ACTIVATE_DATAGRAM_CHECKSUM)
		checkCRC((uint8_t*) msg->buf, msg->size - CHECKSUM_SIZE);
}

inline void checkGpsDatagram(Telemetry_Message *msg, uint32_t ts, uint32_t seq, GPS_data gps) {
	int size = 17; //bytes

	ASSERT_EQ(msg->size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg->buf, GPS_PACKET, ts, seq);

	checkVal8((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER, gps.sats, "Sats number incorrect");

	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 1, gps.hdop, "hdop incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 5, gps.lat, "lat incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 9, gps.lon, "lon incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 13, gps.altitude, "altitude incorrect");

	if(ACTIVATE_DATAGRAM_CHECKSUM)
		checkCRC((uint8_t*) msg->buf, msg->size - CHECKSUM_SIZE);
}

inline void checkStateDatagram(Telemetry_Message *msg, uint32_t ts, uint32_t seq, uint8_t id, float val, uint8_t state) {
	int size = 6; //bytes

	ASSERT_EQ(msg->size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg->buf, STATUS_PACKET, ts, seq);

	checkVal8((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER, id, "ID incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 1, val, "value incorrect");
	checkVal8((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 5, state, "state incorrect");

	if(ACTIVATE_DATAGRAM_CHECKSUM)
		checkCRC((uint8_t*) msg->buf, msg->size - CHECKSUM_SIZE);
}

inline void checkPropulsionDatagram(Telemetry_Message *msg, uint32_t ts, uint32_t seq, PropulsionData prop) {
	int size = 22; //bytes

	ASSERT_EQ(msg->size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg->buf, PROPULSION_DATA_PACKET, ts, seq);

	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 0, prop.pressure1, "pressure 1 incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 4, prop.pressure2, "pressure 2 incorrect");

	checkVal16((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 8, prop.temperature1, "temp 1 incorrect");
	checkVal16((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 10, prop.temperature2, "temp 2 incorrect");
	checkVal16((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 12, prop.temperature3, "temp 3 incorrect");

	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 14, prop.status, "status incorrect");
	checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + 18, prop.motor_position, "motor position incorrect");

	if(ACTIVATE_DATAGRAM_CHECKSUM)
		checkCRC((uint8_t*) msg->buf, msg->size - CHECKSUM_SIZE);
}

#endif //TELEMETRY_TEST_DATAGRAM_CHECK
