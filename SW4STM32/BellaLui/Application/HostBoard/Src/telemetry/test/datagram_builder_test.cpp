/*
 * datagram_builder_test.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Oliver
 */

#include <gtest/gtest.h>
#include "telemetry/test/datagram_check.hpp"
#include "telemetry/datagram_builder.h"
#include "telemetry/telemetry_protocol.h"

// TODO: a test with buffer overflow !

TEST(TelemetryTest, DatagramBuilder_Empty) {
	uint32_t ts = 125, seq = 123;
	uint8_t type = GPS_PACKET;

	DatagramBuilder db(0, type, ts, seq);
	Telemetry_Message *msg = db.finalizeDatagram();

	ASSERT_EQ(msg->size, TOTAL_DATAGRAM_OVERHEAD);
	checkHeader((uint8_t*) msg->buf, type, ts, seq);

	if(ACTIVATE_DATAGRAM_CHECKSUM)
		checkCRC((uint8_t*) msg->buf, msg->size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramBuilder_Write8) {
	uint32_t ts = 108, seq = 255;
	uint8_t type = STATUS_PACKET;
	int size = 4;
	uint8_t vals[] = {0x1d, 0x1e, 0x1f, 0x20};

	DatagramBuilder db(size, type, ts, seq);
	for(int i = 0; i < size; i++)
		db.write8(vals[i]);
	Telemetry_Message *msg = db.finalizeDatagram();

	ASSERT_EQ(msg->size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg->buf, type, ts, seq);

	for(int i = 0; i < size; i++)
		checkVal8((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + i, vals[i], "Payload " + std::to_string(i) + " mismatch");

	if(ACTIVATE_DATAGRAM_CHECKSUM)
		checkCRC((uint8_t*) msg->buf, msg->size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramBuilder_Write16) {
	uint32_t ts = 158, seq = 548;
	uint8_t type = TELEMETRY_PACKET;
	int size = 4, bytePerElem = 2;
	uint16_t vals[] = {0xff1d, 0xff1e, 0xfe1f, 0xfe20};

	DatagramBuilder db(size * bytePerElem, type, ts, seq);
	for(int i = 0; i < size; i++)
		db.write16(vals[i]);
	Telemetry_Message *msg = db.finalizeDatagram();

	ASSERT_EQ(msg->size, TOTAL_DATAGRAM_OVERHEAD + size * bytePerElem);
	checkHeader((uint8_t*) msg->buf, type, ts, seq);

	for(int i = 0; i < size; i++)
		checkVal16((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + bytePerElem*i, vals[i], "Payload " + std::to_string(i) + " mismatch");

	if(ACTIVATE_DATAGRAM_CHECKSUM)
		checkCRC((uint8_t*) msg->buf, msg->size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramBuilder_Write32) {
	uint32_t ts = 5468, seq = 2541;
	uint8_t type = AIRBRAKES_PACKET;
	int size = 4, bytePerElem = 4;
	uint32_t vals[] = {0x0ff0ff1d, 0x0ff0ff1e, 0xf00ffe1f, 0xf00ffe20};

	DatagramBuilder db(size * bytePerElem, type, ts, seq);
	for(int i = 0; i < size; i++)
		db.write32(vals[i]);
	Telemetry_Message *msg = db.finalizeDatagram();

	ASSERT_EQ(msg->size, TOTAL_DATAGRAM_OVERHEAD + size * bytePerElem);
	checkHeader((uint8_t*) msg->buf, type, ts, seq);

	for(int i = 0; i < size; i++)
		checkVal32((uint8_t*) msg->buf + TOTAL_DATAGRAM_HEADER + bytePerElem*i, vals[i], "Payload " + std::to_string(i) + " mismatch");

	if(ACTIVATE_DATAGRAM_CHECKSUM)
		checkCRC((uint8_t*) msg->buf, msg->size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramCreation_Telemetry) {
	uint32_t ts = 1234;
	IMU_data imu = { { 1.1, 2.2, 3.3 }, { 4.4, 5.5, 6.6 } };
	BARO_data baro = { 15.5, 1.05, 5024.6, 0, 0 };
	float speed = 10.1, altitude = baro.altitude;

	Telemetry_Message *msg = createTelemetryDatagram(ts, &imu, &baro, speed, altitude);

	checkTelemetryDatagram(msg, ts, 0, imu, baro, speed, altitude);
}

TEST(TelemetryTest, DatagramCreation_Airbrakes) {
	uint32_t ts = 2345;
	float angle = 12.05;

	Telemetry_Message *msg = createAirbrakesDatagram(ts, angle);

	checkAirbrakesDatagram(msg, ts, 1, angle);
}

TEST(TelemetryTest, DatagramCreation_GPS) {
	uint32_t ts = 3456;
	GPS_data gps = {0.9, 42.42, 5.5, 5025, 4};

	Telemetry_Message *msg = createGPSDatagram(ts, gps);

	checkGpsDatagram(msg, ts, 2, gps);
}

TEST(TelemetryTest, DatagramCreation_State) {
	uint32_t ts = 4567;
	uint8_t id = 2;
	float val = 10.1;
	uint8_t state = 0x08;

	Telemetry_Message *msg = createStateDatagram(ts, id, val, state);

	checkStateDatagram(msg, ts, 3, id, val, state);
}

TEST(TelemetryTest, DatagramCreation_Propulsion) {
	uint32_t ts = 5678;
	PropulsionData prop = {1001, 1002, 20, -45, 12, 8, -42};

	Telemetry_Message *msg = createPropulsionDatagram(ts, &prop);

	checkPropulsionDatagram(msg, ts, 4, prop);
}




