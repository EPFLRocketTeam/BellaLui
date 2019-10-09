/*
 * datagram_builder.h
 *
 *  Created on: 9 May 2018
 *      Author: Clément Nussbaumer
 */

#ifndef MISC_DATAGRAM_BUILDER_H_
#define MISC_DATAGRAM_BUILDER_H_

#include <stdint.h>
#include <Misc/datastructs.h>

#include <stm32f4xx_hal.h>

class DatagramBuilder
{
public:
  DatagramBuilder (uint16_t datagramSize, uint8_t datagramType, uint32_t datagramSequenceNumber);
  void write8 (uint8_t);

  template<typename T>
  void write16 (T val);

  Telemetry_Message finalizeDatagram ();

  template<typename T>
    void write32 (T val);

private:
  void* datagramPtr;

  uint16_t datagramSize, datagramCrc;
  uint16_t currentIdx;

};


template<typename T>
  inline void DatagramBuilder::write16 (T val)
  {
    if (currentIdx + 2 <= datagramSize)
      {
        *(uint16_t*) ((uint8_t*) datagramPtr + currentIdx) = __bswap16 (*((uint16_t*) &val));
        currentIdx += 2;
      }
  }

template<typename T>
  inline void DatagramBuilder::write32 (T val)
  {
    if (currentIdx + 4 <= datagramSize)
      {
        *(uint32_t*) ((uint8_t*) datagramPtr + currentIdx) = __bswap32 (*((uint32_t*) &val));
        currentIdx += 4;
      }
  }

#endif /* MISC_DATAGRAM_BUILDER_H_ */
