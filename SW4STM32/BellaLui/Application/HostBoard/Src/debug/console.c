/*
 * console.c
 *
 *  Created on: Feb 17, 2020
 *      Author: Quentin
 */

#include "debug/console.h"

#include <stdarg.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include <cmsis_os.h>

UART_HandleTypeDef* __console_uart;

static volatile SemaphoreHandle_t console_semaphore;
static char buffer[128];

void rocket_log_lock() {
	xSemaphoreTake(console_semaphore, 10 * portTICK_PERIOD_MS);
}

void rocket_log_release() {
	xSemaphoreGive(console_semaphore);
}

int rocket_boot_log(const char *format, ...) {
	va_list args;
	va_start(args, format);

	if(vsprintf(buffer, format, args) > 0) {
		HAL_UART_Transmit(__console_uart, (uint8_t*) buffer, strlen(buffer), 0xffffff);
	}

	va_end(args);

	return 0;
}

int rocket_log(const char *format, ...) {
	va_list args;
	va_start(args, format);


	char buffer[128];
	if(vsprintf(buffer, format, args) > 0) {
		HAL_UART_Transmit(__console_uart, (uint8_t*) buffer, strlen(buffer), 0xffffff);
	}

	va_end(args);

	osDelay(1);

	return 0;
}

void rocket_log_init(UART_HandleTypeDef* uart) {
	#ifdef DEBUG_MONITOR
	initialise_monitor_handles();
	#endif

	__console_uart = uart;

	console_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(console_semaphore);

	rocket_boot_log("\x1b[2J\x1b[H");
}
