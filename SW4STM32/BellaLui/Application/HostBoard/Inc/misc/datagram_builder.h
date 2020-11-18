/*
 * datagram_builder.h
 *
 *  Created on: 9 May 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef MISC_DATAGRAM_BUILDER_H_
#define MISC_DATAGRAM_BUILDER_H_

#include <stdint.h>
#include <stm32f4xx_hal.h>
#include "../../../HostBoard/Inc/misc/datastructs.h"

#if defined __GNUC__
#define bswap16(x) __builtin_bswap16(x)
#else
#define bswap16(x) ((uint16_t)((((uint16_t) (x) & 0xff00) >> 8) | \
                               (((uint16_t) (x) & 0x00ff) << 8)))
#endif
#endif

#ifndef bswap32
#if defined __GNUC__
#define bswap32(x) __builtin_bswap32(x)
#else
#define bswap32(x) ((uint32_t)((((uint32_t) (x) & 0xff000000) >> 24) | \
                               (((uint32_t) (x) & 0x00ff0000) >> 8) | \
                               (((uint32_t) (x) & 0x0000ff00) << 8) | \
                               (((uint32_t) (x) & 0x000000ff) << 24)))
#endif

class DatagramBuilder
{
public:
  DatagramBuilder (uint16_t datagramSize, uint8_t datagramType, uint32_t timestamp, uint32_t datagramSequenceNumber);

  template<typename T> void write8 (T val);
  template<typename T> void write16 (T val);
  template<typename T> void write32 (T val);

  Telemetry_Message finalizeDatagram ();

private:
  void* datagramPtr;

  uint16_t datagramSize, datagramCrc;
  uint16_t currentIdx;

};

template<typename T> inline void DatagramBuilder::write8 (T val)
{
	if (currentIdx + 1 <= datagramSize)
    {
		*(uint8_t*) ((uint8_t*) datagramPtr + currentIdx++) = val;
    }
}


template<typename T> inline void DatagramBuilder::write16 (T val)
  {
    if (currentIdx + 2 <= datagramSize)
      {
        *(uint16_t*) ((uint8_t*) datagramPtr + currentIdx) = bswap16 (*((uint16_t*) &val));
        currentIdx += 2;
      }
  }

template<typename T> inline void DatagramBuilder::write32 (T val)
  {
    if (currentIdx + 4 <= datagramSize)
      {
        *(uint32_t*) ((uint8_t*) datagramPtr + currentIdx) = bswap32 (*((uint32_t*) &val));
        currentIdx += 4;
      }
  }

#endif /* MISC_DATAGRAM_BUILDER_H_ */
