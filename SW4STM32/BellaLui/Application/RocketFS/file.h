/*
 * file.h
 *
 *  Created on: 19 Oct 2019
 *      Author: Arion
 */

#ifndef INC_FILE_H_
#define INC_FILE_H_

#include <stdint.h>
#include <stdbool.h>


typedef enum FileType { EMPTY, RAW, ECC, CHECKSUM, LOW_REDUNDANCE, HIGH_REDUNDANCE, FOURIER_REDUNDANCE } FileType;

// 32 bytes
typedef struct File {
	char filename[16];
	uint32_t hash;
	uint16_t first_block;
	uint16_t last_block;
	uint32_t length;
	uint16_t used_blocks;
	uint16_t reserved;
} File;


void filename_copy(const char* source, char* target);
bool filename_equals(const char* first, const char* second);

uint32_t hash_filename(const char* identifier);


#endif /* INC_FILE_H_ */
