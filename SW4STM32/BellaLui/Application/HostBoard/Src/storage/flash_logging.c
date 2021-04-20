/*
 * flash_rocket_logging.c
 *
 *  Created on: 22 Oct 2019
 *      Author: Arion
 */

#include "storage/flash_logging.h"
#include "debug/led.h"
#include "debug/console.h"
#include "debug/profiler.h"
#include "debug/monitor.h"

#include "rocket_fs.h"
#include "flash.h"
#include "storage/flash_runtime.h"
#include "storage/heavy_io.h"

#include <cmsis_os.h>
#include <fatfs.h>

#include <stdint.h>
#include <stdbool.h>

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

void stop_logging() {
	is_logging = false;
}

void flash_log(CAN_msg message) {
	/*
	 * Write the CAN message to the front buffer.
	 */

	if(!is_logging && message.id < 50) {
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

	if(!flight_data) {
		flight_data = rocket_fs_newfile(fs, "FLIGHT", RAW);

		if(!flight_data) {
			/*
			 * We failed to create a new file... too many files in the FileSystem?
			 */

			while(true) {
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

		if(!stream.write) {
			/*
			 * Stream is not ready for writing.
			 * An error occurred whilst initialising the stream.
			 */

			while(true) {
				led_set_TK_rgb(led_identifier, 50, 0, 0);
				osDelay(1000);
			}
		}

		/*
		 * Enter the main loop
		 */

		uint32_t bytes_written = 0;
		uint32_t last_update = 0;

		while(!flash_ignore_write) {
			/*
			 * Copy the front buffer to the back buffer and reset the index counter.
			 */

			xSemaphoreTake(master_swap, portMAX_DELAY);

			start_profiler(1);

			if(front_buffer_index > LOGGING_BUFFER_SIZE) {
				front_buffer_index = LOGGING_BUFFER_SIZE;
			}

			for(uint32_t i = 0; i < front_buffer_index; i++) {
				back_buffer[i] = front_buffer[i];
			}

			back_buffer_length = front_buffer_index;
			front_buffer_index = 0;

			xSemaphoreGive(slave_swap);

			/*
			 * Process the back buffer and write its content to the flash memory.
			 */

			start_profiler(2);

			stream.write(back_buffer, back_buffer_length);

			end_profiler();

			bytes_written += back_buffer_length;

			end_profiler();

			if(enter_monitor(FLASH_MONITOR)) {
				uint32_t time = HAL_GetTick();

				rocket_log(" Throughput: %dKB/s\x1b[K\r\n", bytes_written / (time - last_update));
				rocket_log(" Log file size: %dKB\x1b[K\r\n", flight_data->length / 1000);
				rocket_log(" Device capacity: %d%%\x1b[K\r\n", 100 - (100 * flight_data->length / fs->addressable_space));
				exit_monitor(FLASH_MONITOR);

				bytes_written = 0;
				last_update = time;
			}

			if(is_logging) {
				led_set_TK_rgb(led_identifier, 0, 50, 50);
			} else {
				led_set_TK_rgb(led_identifier, 0, 0, 0);
			}
		}

		stream.close();

		rocket_log("Entering passive rocket_logging mode\r\n");

		led_set_TK_rgb(led_identifier, 0, 0, 50);

		xSemaphoreGive(master_io_semaphore);
		xSemaphoreTake(slave_io_semaphore, portMAX_DELAY);

		rocket_log("Entering active rocket_logging mode\r\n");

		led_set_TK_rgb(led_identifier, 0, 0, 0);
	}
}

void acquire_flash_lock() {
	if(!flash_ignore_write) {
		xSemaphoreGive(master_swap);
		flash_ignore_write = true;
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
		rocket_log("Dump failed with error code: %ld\r\n", error_code);
		// An error occurred while copying the flash data into the SD card.
	} else {
		rocket_log("Dump succeeded\r\n");
	}
}

void on_upload_feedback(int32_t error_code) {
	if(error_code != 0) {
		rocket_log("Upload failed with error code: %ld\r\n", error_code);
		// An error occurred while copying the flash data into the SD card.
	}
}

const char* file = "FLIGHT";
void on_dump_request() {
	schedule_heavy_task(&dump_file_on_sd, file, &on_dump_feedback);
}

void on_fullsd_dump_request() {
	schedule_heavy_task(&dump_everything_on_sd, 0, &on_dump_feedback);
}

void on_upload_request(uint16_t block_id) {
	static uint16_t req_blocks[256];
	static uint8_t req_id = 0;

	req_blocks[req_id] = block_id;
	schedule_heavy_task(&upload_block, req_blocks + req_id, &on_upload_feedback);
	req_id++;
}


/*
 * Returns an error code.
 */
int32_t dump_file_on_sd(void* arg) {
	const char* filename = (const char*) arg;
	/*
	 * Stage 1: Initialise the SD output stream.
	 */

	FIL sd_file;

	MX_FATFS_Init();

	osDelay(100);

	if(disk_initialize(0) != 0) {
		return -1; // Failed to initialise disk0
	}

	osDelay(100);

	if(f_mount(&SDFatFS, (TCHAR const*) SDPath, 1) != FR_OK) {
		return -2; // Failed to mount disk0s0
	}

	osDelay(100);

	TCHAR dir[16];
	for(int i = 0; i < 10000; i++) {
		sprintf(dir, "DATA%04d", i);

		FILINFO info;
		if(f_stat(dir, &info) != FR_OK) {
			f_mkdir(dir);
			break;
		}
	}

	TCHAR path[64];

	sprintf(path, "%s/%s.dmp", dir, filename);

	if(f_open(&sd_file, path, FA_OPEN_APPEND | FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
		/* 'STM32.TXT' file Open for write Error */
		return -3;
	}

	/*
	 * Stage 2: Initialise the Flash input stream.
	 */

	acquire_flash_lock(); // Very important call

	FileSystem *fs = get_flash_fs();

	File *flash_file = rocket_fs_getfile(fs, filename);

	if(!flash_file) {
		return -4; // File not found
	}

	Stream stream;
	rocket_fs_stream(&stream, fs, flash_file, OVERWRITE);

	if(!stream.read) {
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

	while(total_bytes_read < flash_file->length && bytes_read > 0) {
		start_profiler(1);
		bytes_read = stream.read(buffer, LOGGING_BUFFER_SIZE);
		end_profiler();

		start_profiler(2);
		f_write(&sd_file, buffer, bytes_read, &bytes_written);
		end_profiler();

		total_bytes_read += bytes_read;
		total_bytes_written += bytes_written;

		rocket_log_lock();
		rocket_log("\e7\x1b[40;0HDumping file... %d%%\x1b[K\e8", 100 * total_bytes_written / flash_file->length);
		rocket_log_release();


		if(bytes_written < 64) {
			// TODO: Disk full: delete old files.
		}

	}

	stream.close();

	rocket_log("\nWrote %ld bytes to the sd card.\r\n", total_bytes_written);

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

	uint8_t attempts = 0;

	while(disk_initialize(0) != 0) {
		if(attempts++ > 5) {
			return -1; // Failed to initialise disk0
		}
	}

	if(f_mount(&SDFatFS, (TCHAR const*) SDPath, 1) != FR_OK) {
		return -2; // Failed to mount disk0s0
	}

	TCHAR dir[16];
	for(int i = 0; i < 10000; i++) {
		sprintf(dir, "DATA%04d", i);

		FILINFO info;
		if(f_stat(dir, &info) != FR_OK) {
			f_mkdir(dir);
			break;
		}
	}

	TCHAR path[64];

	sprintf(path, "%s/FLASH.dmp", dir);

	if(f_open(&sd_file, path, FA_OPEN_APPEND | FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
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

		rocket_log_lock();
		rocket_log("\e7\x1b[40;0HDumping memory... %d%%\x1b[K\e8", 100 * total_bytes_written / (4096 * 4096));
		rocket_log_release();

		if(bytes_written < 2048) {
			// TODO: Disk full: delete old files.
		}
	}

	rocket_log("Dump finished!\r\n");
	rocket_log("\r\nWrote %ld bytes to the sd card.\r\n", total_bytes_written);

	f_sync(&sd_file);
	f_close(&sd_file);

	f_mount(0, 0, 1); // Unmount volume immediately

	release_flash_lock(); // Extremely important call

	return 0;
}

int32_t upload_block(void* arg) {
	uint16_t block = *((uint16_t*) arg);

	acquire_flash_lock();

	rocket_log("----- Block %d begins -----\r\n", block);

	uint8_t buffer[64];
	for(uint8_t i = 0; i < 64; i++) {
		flash_read(block * 4096 + i * 64, buffer, 64);

		for(uint16_t j = 0; j < 64; j++) {
			rocket_log("%02x", buffer[j]);
		}

		osDelay(5); // Let time for the USART unit to process
	}

	rocket_log("\r\n----- Block %d ends -----\r\n", block);

	release_flash_lock();

	return 0;
}
