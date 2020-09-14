/*
 * io_driver.h
 *
 *  Created on: 1 Oct 2019
 *      Author: Arion
 */

#ifndef IO_DRIVER_H_
#define IO_DRIVER_H_



#define IO_TIMEOUT 2000L



#include <stdbool.h>

#include "quadspi.h"


/*
 * Terminal bits error rhythms
 */

#define ERROR_RUN         0b11111111
#define ERROR_POLL        0b10101010
#define ERROR_TRANSMIT    0b10111111
#define ERROR_RECEIVE     0b11111101
#define ERROR_STATE       0b10111101


/*
 * Penultimate bits error rhythms
 */

#define ERROR_READ        (0b11110010 << 8)
#define ERROR_WRITE		  (0b01001111 << 8)
#define ERROR_ERASE		  (0b11100111 << 8)



typedef struct Command {
	QSPI_CommandTypeDef qspi_command;
} Command;

Command get_default_command();


void with_address(Command* cmd, uint32_t address);
void with_data(Command* cmd, uint32_t address);

bool qspi_run(Command* cmd, uint32_t instruction);
bool qspi_poll(Command* cmd, uint32_t instruction, uint8_t bit, bool value);
bool qspi_transmit(uint8_t* buffer);
bool qspi_receive(uint8_t* buffer);

void flash_init();
void flash_read(uint32_t address, uint8_t* buffer, uint32_t length);
void flash_write(uint32_t address, uint8_t* buffer, uint32_t length);
void flash_erase_subsector(uint32_t address);
void flash_erase_sector(uint32_t address);
void flash_erase_all();



#endif /* IO_DRIVER_H_ */
