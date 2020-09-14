/*
 * console.c
 *
 *  Created on: Feb 17, 2020
 *      Author: Arion
 */

#include "debug/console.h"

#include "debug/shell.h"

#include "can_transmission.h"
#include "sync.h"

#include <stdarg.h>
#include <string.h>
#include <stm32f4xx_hal.h>
#include <cmsis_os.h>

UART_HandleTypeDef* __console_uart;

static volatile SemaphoreHandle_t console_semaphore;

static char buffer[CONSOLE_BUFFER_SIZE];

void rocket_log_lock() {
	xSemaphoreTake(console_semaphore, 100 * portTICK_PERIOD_MS);
}

void rocket_log_release() {
	xSemaphoreGive(console_semaphore);
}

void rocket_direct_transmit(uint8_t* buffer, uint32_t length) {
	HAL_UART_Transmit(__console_uart, buffer, length, 0xFFFFFF);
}

void __shell_transmit(uint8_t* buffer, uint32_t length) {
	buffer[length++] = '\0'; // NULL-terminated

	while(length % 4 != 0) {
		buffer[length++] = '\0';
	}

	for(uint8_t i = 0; i < length / 4; i++) {
		uint32_t data = ((uint32_t*) buffer)[i];
		can_setFrame(data, DATA_ID_SHELL_OUTPUT, HAL_GetTick());
	}
}

void rocket_transmit(uint8_t* buffer, uint32_t length) {
	int8_t bridge = get_shell_bridge();

	if(bridge == -1) {
		rocket_direct_transmit(buffer, strlen((char*) buffer));
	} else {
		__shell_transmit(buffer, strlen((char*) buffer));
	}
}

int rocket_boot_log(const char *format, ...) {
	va_list args;
	va_start(args, format);

	if(vsprintf(buffer, format, args) > 0) {
		rocket_transmit((uint8_t*) buffer, strlen(buffer));
	}

	va_end(args);

	return 0;
}

int rocket_log(const char *format, ...) {
	va_list args;
	va_start(args, format);

	if(vsprintf(buffer, format, args) > 0) {
		rocket_transmit((uint8_t*) buffer, strlen(buffer));
	}

	va_end(args);

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

UART_HandleTypeDef* get_console_uart() {
	return __console_uart;
}
