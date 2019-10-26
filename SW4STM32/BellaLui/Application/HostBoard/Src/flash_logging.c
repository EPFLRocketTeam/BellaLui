/*
 * flash_logging.c
 *
 *  Created on: 22 Oct 2019
 *      Author: Arion
 */

#include <stdint.h>
#include <stdbool.h>

#include "flash_logging.h"

#include "cmsis_os.h"
#include "led.h"

#include "rocket_fs.h"
#include "flash.h"


volatile CAN_msg front_buffer[LOGGING_BUFFER_SIZE] = { 0 };
volatile uint32_t front_buffer_index = 0;

volatile SemaphoreHandle_t lock;

void flash_log(CAN_msg message) {
	/*
	 * Write the CAN message to the front buffer.
	 */

	xSemaphoreTake(lock, 1);

	if(front_buffer_index < LOGGING_BUFFER_SIZE) {
		front_buffer[front_buffer_index] = message;
		front_buffer_index++;
	} else {
		// Error: Buffer overflow. Too many CAN messages to process.
	}

	xSemaphoreGive(lock);
}


void __debug(const char* message) {
	printf("%s\n", message);
}


void TK_logging_thread(void const *pvArgs) {
	static CAN_msg back_buffer[LOGGING_BUFFER_SIZE];
	static uint32_t back_buffer_length = 0;

	osDelay(200);

	uint32_t led_identifier = led_register_TK();

	lock = xSemaphoreCreateBinary();

	/*
	 * Allocate, initialise and mount the FileSystem.
	 */

	FileSystem* fs = (FileSystem*) pvPortMalloc(sizeof(FileSystem));

	// rocket_fs_debug(fs, &__debug);
	rocket_fs_device(fs, "NOR Flash", 4096 * 4096, 4096);
	rocket_fs_bind(fs, &flash_read, &flash_write, &flash_erase_subsector);

	rocket_fs_mount(fs);


	/*
	 * Open the 'Flight Data' file or create a new one if it does not exist.
	 */
	File* flight_data = rocket_fs_getfile(fs, "Flight Data");

	if(!flight_data) {
		flight_data = rocket_fs_newfile(fs, "Flight Data", RAW);

		if (!flight_data) {
			/*
			 * We failed to create a new file... too many files in the FileSystem?
			 */

			while (true) {
				led_set_TK_rgb(led_identifier, 50, 0, 0);
				osDelay(1000);
			}
		}
	}

	/*
	 * Initialise a stream pointing to the 'Flight Data' file.
	 */
	Stream stream;
	rocket_fs_stream(&stream, fs, flight_data, APPEND);

	if(!stream.write) {
		/*
		 * Stream is not ready for writing.
		 * An error occurred whilst initialising the stream.
		 */

		while (true) {
			led_set_TK_rgb(led_identifier, 50, 0, 0);
			osDelay(1000);
		}
	}

	/*
	 * We are now ready to handle data.
	 * Unlock the semaphore.
	 */

	xSemaphoreGive(lock);

	/*
	 * Enter the main loop
	 */
	while(true) {
		/*
		 * Copy the front buffer to the back buffer and reset the index counter.
		 */
		xSemaphoreTake(lock, 1);

		for(uint32_t i = 0; i < front_buffer_index; i++) {
			back_buffer[i] = front_buffer[i];
		}

		back_buffer_length = front_buffer_index;
		front_buffer_index = 0;

		xSemaphoreGive(lock);

		/*
		 * Process the back buffer and write its content to the flash memory.
		 */

		CAN_msg current;
		for(uint32_t i = 0; i < back_buffer_length; i++) {
			current = back_buffer[i];

			stream.write32(current.data);
			stream.write8(current.id);
			stream.write32(current.id_CAN);
			stream.write32(current.timestamp);
		}

		led_set_TK_rgb(led_identifier, 0, 50, 50);
	}

	stream.close();
}
