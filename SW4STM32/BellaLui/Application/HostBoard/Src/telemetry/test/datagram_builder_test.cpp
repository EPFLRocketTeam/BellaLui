/*
 * datagram_builder_test.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Oliver
 */

#include <gtest/gtest.h>
#include "telemetry/test/datagram_check.hpp"
#include "misc/datagram_builder.h"
#include "telemetry/telemetry_protocol.h"

// TODO: a test with buffer overflow !

TEST(TelemetryTest, DatagramBuilder_Empty) {
	uint32_t ts = 125, seq = 123;
	uint8_t type = GPS_PACKET;

	DatagramBuilder db(0, type, ts, seq);
	Telemetry_Message msg = db.finalizeDatagram();

	ASSERT_EQ(msg.size, TOTAL_DATAGRAM_OVERHEAD);
	checkHeader((uint8_t*) msg.ptr, type, ts, seq);
	checkCRC((uint8_t*) msg.ptr, msg.size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramBuilder_Write8) {
	uint32_t ts = 108, seq = 255;
	uint8_t type = STATUS_PACKET;
	int size = 4;
	uint8_t vals[] = {0x1d, 0x1e, 0x1f, 0x20};

	DatagramBuilder db(size, type, ts, seq);
	for(int i = 0; i < size; i++)
		db.write8(vals[i]);
	Telemetry_Message msg = db.finalizeDatagram();

	ASSERT_EQ(msg.size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg.ptr, type, ts, seq);

	for(int i = 0; i < size; i++)
		checkVal8((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + i, vals[i], "Payload " + std::to_string(i) + " mismatch");

	checkCRC((uint8_t*) msg.ptr, msg.size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramBuilder_Write16) {
	uint32_t ts = 158, seq = 548;
	uint8_t type = TELEMETRY_PACKET;
	int size = 4, bytePerElem = 2;
	uint16_t vals[] = {0xff1d, 0xff1e, 0xfe1f, 0xfe20};

	DatagramBuilder db(size * bytePerElem, type, ts, seq);
	for(int i = 0; i < size; i++)
		db.write16(vals[i]);
	Telemetry_Message msg = db.finalizeDatagram();

	ASSERT_EQ(msg.size, TOTAL_DATAGRAM_OVERHEAD + size * bytePerElem);
	checkHeader((uint8_t*) msg.ptr, type, ts, seq);

	for(int i = 0; i < size; i++)
			checkVal16((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + bytePerElem*i, vals[i], "Payload " + std::to_string(i) + " mismatch");

	checkCRC((uint8_t*) msg.ptr, msg.size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramBuilder_Write32) {
	uint32_t ts = 5468, seq = 2541;
	uint8_t type = AIRBRAKES_PACKET;
	int size = 4, bytePerElem = 4;
	uint32_t vals[] = {0x0ff0ff1d, 0x0ff0ff1e, 0xf00ffe1f, 0xf00ffe20};

	DatagramBuilder db(size * bytePerElem, type, ts, seq);
	for(int i = 0; i < size; i++)
		db.write32(vals[i]);
	Telemetry_Message msg = db.finalizeDatagram();

	ASSERT_EQ(msg.size, TOTAL_DATAGRAM_OVERHEAD + size * bytePerElem);
	checkHeader((uint8_t*) msg.ptr, type, ts, seq);

	for(int i = 0; i < size; i++)
			checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + bytePerElem*i, vals[i], "Payload " + std::to_string(i) + " mismatch");

	checkCRC((uint8_t*) msg.ptr, msg.size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramCreation_Telemetry) {
	uint32_t ts = 1234;
	IMU_data imu = { { 1.1, 2.2, 3.3 }, { 4.4, 5.5, 6.6 } };
	BARO_data baro = { 15.5, 1.05, 5024.6, 0, 0 };
	float32_t speed = 10.1, altitude = baro.altitude;

	int size = 40; //bytes

	Telemetry_Message msg = createTelemetryDatagram(ts, &imu, &baro, speed, altitude);

	ASSERT_EQ(msg.size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg.ptr, TELEMETRY_PACKET, ts, 0);

	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 0, imu.acceleration.x, "Acceleration X incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 4, imu.acceleration.y, "Acceleration Y incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 8, imu.acceleration.z, "Acceleration Z incorrect");

	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 12, imu.eulerAngles.x, "Euler X incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 16, imu.eulerAngles.y, "Euler Y incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 20, imu.eulerAngles.z, "Euler Z incorrect");

	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 24, baro.temperature, "Temperature incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 28, baro.pressure, "Pressure incorrect");

	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 32, speed, "Speed incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 36, altitude, "Altitude incorrect");

	checkCRC((uint8_t*) msg.ptr, msg.size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramCreation_Airbrakes) {
	uint32_t ts = 2345;
	float32_t angle = 12.05;

	int size = 4; //bytes

	Telemetry_Message msg = createAirbrakesDatagram(ts, angle);

	ASSERT_EQ(msg.size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg.ptr, AIRBRAKES_PACKET, ts, 1);

	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER, angle, "Angle incorrect");

	checkCRC((uint8_t*) msg.ptr, msg.size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramCreation_GPS) {
	uint32_t ts = 3456;
	GPS_data gps = {0.9, 42.42, 5.5, 5025, 4};

	int size = 17; //bytes

	Telemetry_Message msg = createGPSDatagram(ts, gps);

	ASSERT_EQ(msg.size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg.ptr, GPS_PACKET, ts, 2);

	checkVal8((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER, gps.sats, "Sats number incorrect");

	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 1, gps.hdop, "hdop incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 5, gps.lat, "lat incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 9, gps.lon, "lon incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 13, gps.altitude, "altitude incorrect");

	checkCRC((uint8_t*) msg.ptr, msg.size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramCreation_State) {
	uint32_t ts = 4567;
	uint8_t id = 1;
	float32_t val = 10.1;
	uint8_t state = 0x08;

	int size = 6; //bytes

	Telemetry_Message msg = createStateDatagram(ts, id, val, state);

	ASSERT_EQ(msg.size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg.ptr, STATUS_PACKET, ts, 3);

	checkVal8((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER, id, "ID incorrect");
	checkVal32((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 1, val, "value incorrect");
	checkVal8((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 5, state, "state incorrect");

	checkCRC((uint8_t*) msg.ptr, msg.size - CHECKSUM_SIZE);
}

TEST(TelemetryTest, DatagramCreation_Propulsion) {
	uint32_t ts = 5678;
	PropulsionData prop = {1001, 1002, 20, -45, 12, 8, -42};

	int size = 14; //bytes

	Telemetry_Message msg = createPropulsionDatagram(ts, &prop);

	ASSERT_EQ(msg.size, TOTAL_DATAGRAM_OVERHEAD + size);
	checkHeader((uint8_t*) msg.ptr, PROPULSION_DATA_PACKET, ts, 4);

	checkVal16((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 0, prop.pressure1, "pressure 1 incorrect");
	checkVal16((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 2, prop.pressure2, "pressure 2 incorrect");

	checkVal16((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 4, prop.temperature1, "temp 1 incorrect");
	checkVal16((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 6, prop.temperature2, "temp 2 incorrect");
	checkVal16((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 8, prop.temperature3, "temp 3 incorrect");

	checkVal16((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 10, prop.status, "status incorrect");
	checkVal16((uint8_t*) msg.ptr + TOTAL_DATAGRAM_HEADER + 12, prop.motor_position, "motor position incorrect");

	checkCRC((uint8_t*) msg.ptr, msg.size - CHECKSUM_SIZE);
}




