/*
 * datagram_builder.cpp
 *
 *  Created on: 9 May 2018
 *      Author: Clément Nussbaumer
 */

#include <Telemetry/telemetry_protocol.h>
#include "Telemetry/simpleCRC.h"

#include <cmsis_os.h>
#include <Misc/datagram_builder.h>
#include <stm32f4xx_hal.h>

#define MALLOC_SIZE 64 // same malloc size for all datagrams, otherwise it fragments the memory

DatagramBuilder::DatagramBuilder (uint16_t datagramPayloadSize, uint8_t datagramType, uint32_t datagramSequenceNumber) :
    datagramSize (datagramPayloadSize + TOTAL_DATAGRAM_OVERHEAD)
{
  datagramPtr = pvPortMalloc (MALLOC_SIZE);
  if (datagramPtr != NULL)
    {
      currentIdx = 0;
    }
  else
    {
      currentIdx = datagramSize; // prevents from writing anything.
      return;
    }

  // Beginning of telemetry packet:
  for (size_t i = 0; i < PREAMBLE_SIZE; ++i)
    {
      write8 (HEADER_PREAMBLE_FLAG);
    }

  //DATAGRAM SEQ NUMBER
  write32<uint32_t> (datagramSequenceNumber);

  //PAYLOAD TYPE
  write8 (datagramType);

  datagramCrc = CRC_16_GENERATOR_POLY.initialValue;
  for (int16_t i = currentIdx - 5; i < currentIdx; i++)
    {
      //Calculate checksum for datagram seq number and payload type fields
      datagramCrc = CalculateRemainderFromTable (*((uint8_t*) datagramPtr + i), datagramCrc);
    }

  //CONTROL FLAG
  write8 (CONTROL_FLAG);

}

void DatagramBuilder::write8 (uint8_t val)
{
  if (currentIdx + 1 <= datagramSize)
    {
      *(uint8_t*) ((uint8_t*) datagramPtr + currentIdx++) = val;
    }
}

Telemetry_Message DatagramBuilder::finalizeDatagram ()
{
  for (int16_t i = (PREAMBLE_SIZE + HEADER_SIZE + CONTROL_FLAG_SIZE); i < currentIdx; i++)
    {
      //Calculate checksum for datagram and payload fields
      datagramCrc = CalculateRemainderFromTable (*((uint8_t*) datagramPtr + i), datagramCrc);
    }

  datagramCrc = FinalizeCRC (datagramCrc);
  write16 (datagramCrc);

  Telemetry_Message m =
    { .ptr = datagramPtr, .size = datagramSize };

  return m;
}
