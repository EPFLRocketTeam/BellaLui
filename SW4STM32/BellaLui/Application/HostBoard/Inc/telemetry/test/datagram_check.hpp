/*
 * datagram_check.h
 *
 *  Created on: 26 Feb 2021
 *      Author: Oliver
 */

#ifndef TELEMETRY_TEST_DATAGRAM_CHECK
#define TELEMETRY_TEST_DATAGRAM_CHECK

#include <gtest/gtest.h>
#include "misc/datagram_builder.h"
#include "telemetry/simpleCRC.h"

void checkVal8(uint8_t *ptr, uint8_t expected, std::string msg) {
	EXPECT_EQ(*ptr, expected) << msg;
}

void checkVals(uint8_t *ptr, uint8_t *expected, int size, std::string msg) {
	for(int i = 0; i < size; i++) {
		checkVal8(ptr + i, expected[i], msg + " (index " + std::to_string(i) + ")");
	}
}

void checkVal16(uint8_t *ptr, uint16_t expected, std::string msg) {
	uint16_t swappedVal = bswap16(expected);
	uint8_t *arrByte = (uint8_t*) &swappedVal;
	checkVals(ptr, arrByte, 2, msg);
}

void checkVal32(uint8_t *ptr, uint32_t expected, std::string msg) {
	uint32_t swappedVal = bswap32(expected);
	uint8_t *arrByte = (uint8_t*) &swappedVal;
	checkVals(ptr, arrByte, 4, msg);
}


void checkHeader(uint8_t *ptr, uint8_t id, uint32_t ts, uint32_t seq) {
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

void checkCRC(uint8_t *ptr, int size) {
	uint16_t crc = CRC_16_GENERATOR_POLY.initialValue;

	for(int i = 0; i < size; i++) {
		crc = CalculateRemainderFromTable(ptr[i], crc);
	}
	crc = FinalizeCRC(crc);

	checkVal16(ptr + size, crc, "CRC mismatch");
}

#endif //TELEMETRY_TEST_DATAGRAM_CHECK
