/*
 * flash_rocket_logging.c
 *
 *  Created on: 22 Oct 2019
 *      Author: Arion
 */

#include <storage/flash_logging.h>

#include "cmsis_os.h"
#include "fatfs.h"


#include <stdint.h>
#include <stdbool.h>


#include <debug/led.h>
#include <debug/console.h>

#include <rocket_fs.h>
#include <flash.h>
#include <storage/flash_runtime.h>
#include <storage/heavy_io.h>



static volatile uint8_t front_buffer[LOGGING_BUFFER_SIZE] = { 0 };
static volatile uint32_t front_buffer_index = 0;

static volatile SemaphoreHandle_t master_swap;
static volatile SemaphoreHandle_t slave_swap;

static volatile SemaphoreHandle_t master_io_semaphore;
static volatile SemaphoreHandle_t slave_io_semaphore;
static volatile bool flash_ignore_write = false;

static volatile bool is_logging = false;

void init_logging() {
   master_swap = xSemaphoreCreateBinary();
   slave_swap = xSemaphoreCreateBinary();
   master_io_semaphore = xSemaphoreCreateBinary();
   slave_io_semaphore = xSemaphoreCreateBinary();
}

void start_logging() {
	is_logging = true;
}

void flash_log(CAN_msg message) {
	/*
	 * Write the CAN message to the front buffer.
	 */

	if(!is_logging) {
		return;
	}

	if(front_buffer_index <= LOGGING_BUFFER_SIZE - 8) {
		front_buffer[front_buffer_index++] = (uint8_t) (message.id);
		front_buffer[front_buffer_index++] = (uint8_t) (message.timestamp >> 16);
		front_buffer[front_buffer_index++] = (uint8_t) (message.timestamp >> 8);
		front_buffer[front_buffer_index++] = (uint8_t) (message.timestamp >> 0);
		front_buffer[front_buffer_index++] = (uint8_t) (message.data >> 24);
		front_buffer[front_buffer_index++] = (uint8_t) (message.data >> 16);
		front_buffer[front_buffer_index++] = (uint8_t) (message.data >> 8);
		front_buffer[front_buffer_index++] = (uint8_t) (message.data >> 0);
	}

	// vTaskDelay(400 * portTICK_PERIOD_MS / 1000);  // Add delay to smoothen the thread blocking time due to flash synchronisation

	if(front_buffer_index >= LOGGING_BUFFER_SIZE) {
		xSemaphoreGive(master_swap);
		xSemaphoreTake(slave_swap, 10 * portTICK_PERIOD_MS);
	}
}

void TK_logging_thread(void const *pvArgs) {
	static uint8_t back_buffer[LOGGING_BUFFER_SIZE];
	static uint32_t back_buffer_length = 0;

	uint32_t led_identifier = led_register_TK();

	FileSystem *fs = get_flash_fs();

	/*
	 * Allocate, initialise and mount the FileSystem.
	 */

	/*
	 * Open the 'FlightData' file or create a new one if it does not exist.
	 */
	File *flight_data = rocket_fs_getfile(fs, "FLIGHT");

	if (!flight_data) {
		flight_data = rocket_fs_newfile(fs, "FLIGHT", RAW);

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
		 * Initialise a stream pointing to the 'FlightData' file.
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
		 * Enter the main loop
		 */

		uint32_t can = 0;

		while (!flash_ignore_write) {
			/*
			 * Copy the front buffer to the back buffer and reset the index counter.
			 */

			xSemaphoreTake(master_swap, portMAX_DELAY);

			if(front_buffer_index > LOGGING_BUFFER_SIZE) {
				front_buffer_index = LOGGING_BUFFER_SIZE;
			}

			for (uint32_t i = 0; i < front_buffer_index; i++) {
				back_buffer[i] = front_buffer[i];
			}

			back_buffer_length = front_buffer_index;
			front_buffer_index = 0;

			xSemaphoreGive(slave_swap);

			/*
			 * Process the back buffer and write its content to the flash memory.
			 */

			stream.write(back_buffer, back_buffer_length);
			can += back_buffer_length;

			rocket_log("Wrote 2KB worth of CAN messages.\n");

			led_set_TK_rgb(led_identifier, 0, 50, 50);
		}

		stream.close();

		rocket_log("Entering passive rocket_logging mode\n");

		led_set_TK_rgb(led_identifier, 0, 0, 50);

		xSemaphoreGive(master_io_semaphore);
		xSemaphoreTake(slave_io_semaphore, portMAX_DELAY);

		rocket_log("Entering active rocket_logging mode\n");
	}
}


void acquire_flash_lock() {
	if(!flash_ignore_write) {
		flash_ignore_write = true;
		xSemaphoreGive(master_swap);
		xSemaphoreTake(slave_swap, portMAX_DELAY);
		xSemaphoreTake(master_io_semaphore, portMAX_DELAY);
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
		rocket_log("Dump failed with error code: %ld\n", error_code);
		// An error occurred while copying the flash data into the SD card.
	} else {
      rocket_log("Dump succeeded\n");
	}
}

void on_dump_request() {
	schedule_heavy_task(&dump_file_on_sd, "FLIGHT", &on_dump_feedback);
}

void on_fullsd_dump_request() {
   schedule_heavy_task(&dump_everything_on_sd, 0, &on_dump_feedback);
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

	if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 1) != FR_OK) {
		return -2; // Failed to mount disk0s0
	}

	TCHAR dir[16];
	for (int i = 0; i < 10000; i++) {
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

	uint8_t buffer[LOGGING_BUFFER_SIZE];
	uint32_t total_bytes_read = 0;
	uint32_t total_bytes_written = 0;

   int32_t bytes_read = 1;
   UINT bytes_written = 0;

   rocket_fs_touch(fs, flash_file);

   rocket_log("Dumping file...\n");

	while(total_bytes_read < flash_file->length && bytes_read > 0) {
	   bytes_read = stream.read(buffer, LOGGING_BUFFER_SIZE);

		f_write(&sd_file, buffer, bytes_read, &bytes_written);

		total_bytes_read += bytes_read;
		total_bytes_written += bytes_written;

		if(bytes_written < 64) {
			// TODO: Disk full: delete old files.
		}
	}

	stream.close();

	rocket_log("Wrote %ld bytes to the sd card.\n", total_bytes_written);

	f_sync(&sd_file);
	f_close(&sd_file);

	f_mount(0, 0, 1); // Unmount volume immediately

	release_flash_lock(); // Extremely important call

	return 0;
}

int32_t dump_everything_on_sd(void* arg) {
   /*
    * Stage 1: Initialise the SD output stream.
    */

   FIL sd_file;

   MX_FATFS_Init();

   if (disk_initialize(0) != 0) {
      return -1; // Failed to initialise disk0
   }

   if (f_mount(&SDFatFS, (TCHAR const*) SDPath, 1) != FR_OK) {
      return -2; // Failed to mount disk0s0
   }

   TCHAR dir[16];
   for (int i = 0; i < 10000; i++) {
      sprintf(dir, "DATA%04d", i);

      FILINFO info;
      if (f_stat(dir, &info) != FR_OK) {
         f_mkdir(dir);
         break;
      }
   }

   TCHAR path[64];

   sprintf(path, "%s/FLASH.dmp", dir);

   if (f_open(&sd_file, path,
         FA_OPEN_APPEND | FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
      /* 'STM32.TXT' file Open for write Error */
      return -3;
   }


   /*
    * Stage 2: Initialise the Flash input stream.
    */

   acquire_flash_lock(); // Very important call

   /*
    * Stage 3: Transfer data from Flash to SD card.
    */

   uint8_t buffer[2048];
   uint32_t total_bytes_written = 0;
   UINT bytes_written = 0;

   for(int32_t address = 0; address < 4096 * 4096; address += 2048) {
      flash_read(address, buffer, 2048);
      f_write(&sd_file, buffer, 2048, &bytes_written);

      total_bytes_written += bytes_written;

      if(bytes_written < 2048) {
         // TODO: Disk full: delete old files.
      }
   }

   rocket_log("Wrote %ld bytes to the sd card.\n", total_bytes_written);

   f_sync(&sd_file);
   f_close(&sd_file);

   f_mount(0, 0, 1); // Unmount volume immediately

   release_flash_lock(); // Extremely important call

   return 0;
}
