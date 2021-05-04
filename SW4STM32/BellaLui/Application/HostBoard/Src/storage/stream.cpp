/*
 * stream.c
 *
 *  Created on: 17 Oct 2019
 *      Author: Arion
 */

#include "stream.h"

#include "block_management.h"




bool init_stream(Stream* stream, FileSystem* fs, uint32_t base_address, FileType type) {
	if(!stream->open) {
		stream->fs = fs;
		stream->read_address = base_address;
		stream->write_address = base_address;
		stream->type = type;
		stream->open = true;
		stream->eof = false;

		return true;
	} else {
		fs->log("Error: Stream is already open.");
		return false;
	}

}

Stream::Stream() : fs(0), type(RAW), eof(false), open(false), read_address(0xFFFFFFFFL), write_address(0xFFFFFFFFL) {
	;
}

void Stream::close() {
	read_address = 0xFFFFFFFFL;
	write_address = 0xFFFFFFFFL;
	open = false;

	rocket_fs_flush(fs);

	fs = 0;
}

/* RAW IO FUNCTIONS */
static uint8_t coder[8]; // Used as encoder and decoder

extern "C" void cread(FileSystem* fs, uint32_t address, uint8_t* buffer, uint32_t length) {
	fs->read(address, buffer, length);
}

int32_t Stream::read(uint8_t* buffer, uint32_t length) {
	uint32_t index = 0;
	int32_t readable_length = 0;

	do {
	   readable_length = rfs_access_memory(fs, &read_address, length - index, READ); // Transforms the write address (or fails if end of file) if we are at the end of a readable section

	   if(readable_length <= 0) {
         eof = true;
         return index;
      } else {
    	  eof = false;
      }

	   cread(fs, read_address, buffer, readable_length);


		index += readable_length;
		read_address += readable_length;
	} while(index < length);

	return length;
}

uint8_t Stream::read8() {
	read(coder, 1);
	return coder[0];
}

uint16_t Stream::read16() {
	read(coder, 2);

	uint64_t composition = 0ULL;

	composition |= (uint16_t) coder[1] << 8;
	composition |= (uint16_t) coder[0];

	return composition;
}

uint32_t Stream::read32() {
	read(coder, 4);

	uint64_t composition = 0ULL;

	composition |= (uint64_t) coder[3] << 24;
	composition |= (uint64_t) coder[2] << 16;
	composition |= (uint64_t) coder[1] << 8;
	composition |= (uint64_t) coder[0];

	return composition;
}

uint64_t Stream::read64() {
	read(coder, 8);

	uint64_t composition = 0ULL;

	composition |= (uint64_t) coder[7] << 56;
	composition |= (uint64_t) coder[6] << 48;
	composition |= (uint64_t) coder[5] << 40;
	composition |= (uint64_t) coder[4] << 32;
	composition |= (uint64_t) coder[3] << 24;
	composition |= (uint64_t) coder[2] << 16;
	composition |= (uint64_t) coder[1] << 8;
	composition |= (uint64_t) coder[0];

	return composition;
}

#include <stdio.h>

void Stream::write(uint8_t* buffer, uint32_t length) {
   uint32_t index = 0;
   int32_t writable_length = 0;

   do {
      writable_length = rfs_access_memory(fs, &write_address, length - index, WRITE); // Transforms the write address (or fails if end of file) if we are at the end of a readable section

      if(writable_length <= 0) {
         eof = true;
         return;
      } else {
         eof = false;
      }

      fs->write(write_address, buffer + index, writable_length);

      index += writable_length;
      write_address += writable_length;
   } while(index < length);
}

void Stream::write8(uint8_t data) {
	write(&data, 1);
}

void Stream::write16(uint16_t data) {
	coder[0] = data;
	coder[1] = data >> 8;

	write(coder, 2);
}

void Stream::write32(uint32_t data) {
	coder[0] = data;
	coder[1] = data >> 8;
	coder[2] = data >> 16;
	coder[3] = data >> 24;

	write(coder, 4);
}

void Stream::write64(uint64_t data) {
	coder[0] = data;
	coder[1] = data >> 8;
	coder[2] = data >> 16;
	coder[3] = data >> 24;
	coder[4] = data >> 32;
	coder[5] = data >> 40;
	coder[6] = data >> 48;
	coder[7] = data >> 56;

	write(coder, 8);
}
