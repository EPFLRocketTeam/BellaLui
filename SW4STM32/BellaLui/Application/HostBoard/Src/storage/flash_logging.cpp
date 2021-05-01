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

#include "storage/flash_runtime.h"
#include "storage/heavy_io.h"
#include "rocket_fs.h"
#include "flash.h"

#include <cmsis_os.h>

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
	} else {
		xSemaphoreGive(master_swap);
		xSemaphoreTake(slave_swap, 10 * portTICK_PERIOD_MS);
	}

	// vTaskDelay(400 * portTICK_PERIOD_MS / 1000);  // Add delay to smoothen the thread blocking time due to flash synchronisation
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

			rocket_fs_flush(fs);

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
