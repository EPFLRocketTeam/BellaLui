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



