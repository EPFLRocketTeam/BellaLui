/*
 * flash_logging.c
 *
 *  Created on: 22 Oct 2019
 *      Author: Arion
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "flash_logging.h"

#include "cmsis_os.h"
#include "fatfs.h"

#include "led.h"

#include "rocket_fs.h"
#include "heavy_io.h"


static volatile CAN_msg front_buffer[LOGGING_BUFFER_SIZE] = { 0 };
static volatile uint32_t front_buffer_index = 0;

static volatile SemaphoreHandle_t buffer_lock;

static volatile SemaphoreHandle_t master_io_semaphore;
static volatile SemaphoreHandle_t slave_io_semaphore;
static volatile bool flash_ignore_write = false;


void flash_log(CAN_msg message) {
	/*
	 * Write the CAN message to the front buffer.
	 */

	if(xSemaphoreTake(buffer_lock, 20) == pdTRUE) {
		if (front_buffer_index < LOGGING_BUFFER_SIZE) {
			front_buffer[front_buffer_index] = message;
			front_buffer_index++;
		} else {
			// Error: Buffer overflow. Too many CAN messages to process.
		}

		xSemaphoreGive(buffer_lock);
	}
}

void TK_logging_thread(void const *pvArgs) {
	static CAN_msg back_buffer[LOGGING_BUFFER_SIZE];
	static uint32_t back_buffer_length = 0;

	osDelay(200);

	uint32_t led_identifier = led_register_TK();

	FileSystem *fs = get_flash_fs();

	buffer_lock = xSemaphoreCreateMutex();
	master_io_semaphore = xSemaphoreCreateBinary();
	slave_io_semaphore = xSemaphoreCreateBinary();

	/*
	 * Allocate, initialise and mount the FileSystem.
	 */

	/*
	 * Open the 'Flight Data' file or create a new one if it does not exist.
	 */
	File *flight_data = rocket_fs_getfile(fs, "Flight Data");

	if (!flight_data) {
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

	while(true) {
		/*
		 * Initialise a stream pointing to the 'Flight Data' file.
		 */
		Stream stream;
		rocket_fs_stream(&stream, fs, flight_data, APPEND);

		if (!stream.write) {
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

		xSemaphoreGive(buffer_lock);

		/*
		 * Enter the main loop
		 */
		while (!flash_ignore_write) {
			/*
			 * Copy the front buffer to the back buffer and reset the index counter.
			 */
			xSemaphoreTake(buffer_lock, portMAX_DELAY);

			for (uint32_t i = 0; i < front_buffer_index; i++) {
				back_buffer[i] = front_buffer[i];
			}

			back_buffer_length = front_buffer_index;
			front_buffer_index = 0;

			xSemaphoreGive(buffer_lock);

			/*
			 * Process the back buffer and write its content to the flash memory.
			 */

			CAN_msg current;
			for (uint32_t i = 0; i < back_buffer_length; i++) {
				current = back_buffer[i];

				stream.write32(current.data);
				stream.write8(current.id);
				stream.write32(current.id_CAN);
				stream.write32(current.timestamp);
			}

			led_set_TK_rgb(led_identifier, 0, 50, 50);
		}

		stream.close();

		xSemaphoreGive(master_io_semaphore);
		xSemaphoreTake(slave_io_semaphore, portMAX_DELAY);

		/*
		 * Reopen the stream
		 */
		rocket_fs_stream(&stream, fs, flight_data, APPEND);

		if (!stream.write) {
			while (true) {
				led_set_TK_rgb(led_identifier, 50, 0, 0);
				osDelay(1000);
			}
		}
	}
}


void acquire_flash_lock() {
	if(!flash_ignore_write) {
		flash_ignore_write = true;
		xSemaphoreTake(master_io_semaphore, 10);
	}
}

void release_flash_lock() {
	if(flash_ignore_write) {
		flash_ignore_write = false;
		xSemaphoreGive(slave_io_semaphore);
	}
}


void on_dump_feedback(int32_t error_code) {
	if(error_code != 0) {
		// An error occurred while copying the flash data into the SD card.
	}
}

void on_dump_request() {
	schedule_heavy_task(&dump_file_on_sd, "Flight Data", &on_dump_feedback);
}

/*
 * Returns an error code.
 */
int32_t dump_file_on_sd(const char* filename) {
	/*
	 * Stage 1: Initialise the SD output stream.
	 */

	FIL sd_file;

	MX_FATFS_Init();

	if (disk_initialize(0) != 0) {
		return -1; // Failed to initialise disk0
	}

	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK) {
		return -2; // Failed to mount disk0s0
	}

	TCHAR dir[16];
	for (int i = 0; i < 65536; i++) {
		sprintf(dir, "DATA%04d", i);

		FILINFO info;
		if (f_stat(dir, &info) != FR_OK) {
			f_mkdir(dir);
			break;
		}
	}

	TCHAR path[64];

	sprintf(path, "%s/%s.dmp", dir, filename);

	if (f_open(&sd_file, path,
			FA_OPEN_APPEND | FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
		/* 'STM32.TXT' file Open for write Error */
		return -3;
	}


	/*
	 * Stage 2: Initialise the Flash input stream.
	 */

	acquire_flash_lock(); // Very important call

	FileSystem* fs = get_flash_fs();

	File* flash_file = rocket_fs_getfile(fs, filename);

	if(!flash_file) {
		return -4; // File not found
	}

	Stream stream;
	rocket_fs_stream(&stream, fs, flash_file, OVERWRITE);

	if (!stream.read) {
		/*
		 * Stream is not ready for reading.
		 * An error occurred whilst initialising the stream.
		 */
		return -5;
	}


	/*
	 * Stage 3: Transfer data from Flash to SD card.
	 */

	uint8_t buffer[256];
	UINT bytes_written;

	while(!stream.eof) {
		int32_t bytes_read = stream.read(buffer, 256);
		bytes_written = 0;

		f_write(&sd_file, buffer, bytes_read, &bytes_written);

		if(bytes_written < 256) {
			// TODO: Disk full: delete old files.
		}
	}


	release_flash_lock(); // Extremely important call

	return 0;
}
