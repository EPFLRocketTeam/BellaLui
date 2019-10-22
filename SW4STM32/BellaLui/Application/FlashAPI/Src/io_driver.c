/*
 * io_driver.c
 *
 *  Created on: 1 Oct 2019
 *      Author: Arion
 */

#include "io_driver.h"
#include "flash_state.h"
#include "MT25QL128ABA.h"


/*
 * Reads the flag status register and returns the value of the 8-bits register
 */
uint8_t __read_flags() {
	Command cmd = get_default_command();
	with_data(&cmd, 1);

	if(!qspi_run(&cmd, READ_FLAG_STATUS_REGISTER)) {
		flash_fatal(ERROR_READ | ERROR_RUN);
	}

	uint8_t flags;

	if(!qspi_receive(&flags)) {
		flash_fatal(ERROR_READ | ERROR_RECEIVE);
	}

	return flags;
}



/*
 * Enables the write latch.
 * This function must be called before each PROGRAM or ERASE operation.
 */
bool __write_enable_latch() {
	Command cmd = get_default_command();

	if(qspi_run(&cmd, WRITE_ENABLE_LATCH)) {
		cmd = get_default_command();

		with_data(&cmd, 1);

		if(qspi_poll(&cmd, READ_STATUS_REGISTER, 1, true)) {
			return true;
		}
	}

	return false;
}

/*
 * Call this function to prevent data corruption when a hardware fault (e.g. protection fault) occurs.
 * Please refer to the documentation for details.
 */
bool __write_disable_latch() {
	Command cmd = get_default_command();
	return qspi_run(&cmd, WRITE_ENABLE_LATCH);
}



/*
 *
 * --- Read operations ---
 *
 * Test providers:
 * 	 - read_ut.c
 *
 */

void flash_read(uint32_t address, uint8_t* buffer, uint32_t length) {
	Command cmd = get_default_command();
	with_address(&cmd, address);
	with_data(&cmd, length);


	if(!qspi_run(&cmd, READ_SINGLE)) {
		flash_fatal(ERROR_READ | ERROR_RUN);
	}

	if(!qspi_receive(buffer)) {
		flash_fatal(ERROR_READ | ERROR_RECEIVE);
	}
}



/*
 *
 * --- Write operations ---
 *
 * Test providers:
 * 	 - write_ut.c
 *
 */

void __flash_write_page(uint32_t address, uint8_t* buffer, uint32_t length) {
	__write_enable_latch();

	Command cmd = get_default_command();

	with_address(&cmd, address);
	with_data(&cmd, length);

	if(!qspi_run(&cmd, WRITE_SINGLE)) {
		flash_fatal(ERROR_WRITE | ERROR_RUN);
	}

	if(!qspi_transmit(buffer)) {
		flash_fatal(ERROR_WRITE | ERROR_TRANSMIT);
	}

	/*
	 * Checks if the controller is ready to proceed to the next command
	 */
	cmd = get_default_command();
	with_data(&cmd, 1);

	if(!qspi_poll(&cmd, READ_FLAG_STATUS_REGISTER, 7, true)) {
		flash_fatal(ERROR_ERASE | ERROR_STATE);
	}

	uint8_t flags = __read_flags();

	// Checks if the protection fault flag is set
	if(flags & (1 << 4)) {
		__write_disable_latch(); // Manually reset the latch

		flash_fatal(ERROR_WRITE | ERROR_STATE);
	}
}

void flash_write(uint32_t address, uint8_t* buffer, uint32_t length) {
	while(length > PAGE_SIZE) {
		__flash_write_page(address, buffer, PAGE_SIZE);
		buffer += PAGE_SIZE;
		address += PAGE_SIZE;
		length -= PAGE_SIZE;
	}

	__flash_write_page(address, buffer, length);
}



/*
 *
 * --- Erase operations ---
 *
 * Test providers:
 * 	 - erase_ut.c
 *
 */

void __flash_erase(uint32_t instruction, uint32_t address) {

	__write_enable_latch();


	Command cmd = get_default_command();
	with_address(&cmd, address);


	if(!qspi_run(&cmd, ERASE_SUBSECTOR)) {
		flash_fatal(ERROR_ERASE | ERROR_RUN);
	}

	/*
	 * Checks if the controller is ready to proceed to the next command
	 */
	cmd = get_default_command();
	with_data(&cmd, 1);

	if(!qspi_poll(&cmd, READ_FLAG_STATUS_REGISTER, 7, true)) {
		flash_fatal(ERROR_ERASE | ERROR_STATE);
	}

	/*
	 * Checks if the protection fault flag is set
	 */
	uint8_t flags = __read_flags();

	if(flags & (1 << 5)) {
		__write_disable_latch(); // Manually reset the latch

		flash_fatal(ERROR_ERASE | ERROR_STATE);
	}
}

/*
 * Erases the whole sector represented by the provided address.
 * The address may be any of those within the sector.
 */
void flash_erase_sector(uint32_t address) {
	__flash_erase(ERASE_SECTOR, address);
}


/*
 * Erases the whole sub-sector represented by the provided address.
 * The address may be any of those within the sub-sector.
 */
void flash_erase_subsector(uint32_t address) {
	__flash_erase(ERASE_SUBSECTOR, address);
}
