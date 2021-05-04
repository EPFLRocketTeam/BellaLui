/*
 * file.c
 *
 *  Created on: 19 Oct 2019
 *      Author: Arion
 */

#include "file.h"


void filename_copy(const char* source, char* target) {
	uint8_t i;

   for(i = 0; i < 15; i++) {
		if(source[i] != '\0') {
			target[i] = source[i];
		} else {
         break;
		}
	}

   while(i < 15) {
      target[i++] = '\0';
   }
}

bool filename_equals(const char* first, const char* second) {
	for(uint8_t i = 0; i < 16; i++) {
		if(first[i] != second[i]) {
			return false;
		}
	}

	return true;
}


uint32_t hash_filename(const char* name) {
	uint32_t hash = 13;

	for(uint8_t i = 0; i < 16; i++) {
	   if(name[i] != '\0') {
         hash = 31 * hash + name[i]; // Java JDK8 String.hashCode() implementation
	   } else {
	      break;
	   }
	}

	return hash;
}
