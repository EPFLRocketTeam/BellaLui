/*
 * flash.h
 *
 *  Created on: 1 Oct 2019
 *      Author: Arion
 */

#ifndef FLASH_H_
#define FLASH_H_


#include <stdint.h>


void flash_init();
void flash_read(uint32_t address, uint8_t* buffer, uint32_t length);
void flash_write(uint32_t address, uint8_t* buffer, uint32_t length);
void flash_write_fast(uint32_t address, uint8_t* buffer, uint32_t length);
void flash_erase_subsector(uint32_t address);
void flash_erase_sector(uint32_t address);
void flash_erase_all();

#endif /* FLASH_H_ */
