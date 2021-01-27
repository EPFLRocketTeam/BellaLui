/*
 * datagram_builder.cpp
 *
 *  Created on: 9 May 2018
 *      Author: Cl�ment Nussbaumer
 */

#include "misc/datagram_builder.h"

#include "telemetry/simpleCRC.h"
#include "telemetry/telemetry_protocol.h"
#include "debug/console.h"

#include <stm32f4xx_hal.h>
#include <cmsis_os.h>

#define MALLOC_SIZE 256 // same malloc size for all datagrams, otherwise it fragments the memory

DatagramBuilder::DatagramBuilder(uint16_t datagramPayloadSize, uint8_t datagramType, uint32_t timestamp, uint32_t seqNumber) :
		datagramSize(datagramPayloadSize + TOTAL_DATAGRAM_OVERHEAD) {
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
	for(int16_t i = (HEADER_SIZE /*+ PREAMBLE_SIZE + CONTROL_FLAG_SIZE*/); i < currentIdx; i++) {
		//Calculate checksum for datagram and payload fields
		datagramCrc = CalculateRemainderFromTable(*((uint8_t*) datagramPtr + i), datagramCrc);
	}

	datagramCrc = FinalizeCRC(datagramCrc);
	write16(datagramCrc);

	Telemetry_Message m = { .ptr = datagramPtr, .size = datagramSize };

	return m;
}
