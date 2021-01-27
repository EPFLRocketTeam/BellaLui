/*
 * shell.h
 *
 *  Created on: 5 Sep 2020
 *      Author: arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_DEBUG_SHELL_H_
#define APPLICATION_HOSTBOARD_INC_DEBUG_SHELL_H_

#include <stdint.h>
#include <stdbool.h>

#include <stm32f4xx_hal.h>

#define CMD_BUFFER_SIZE 512
#define CMD_MAX_COMPONENTS 8

struct CommandComponent {
	const char* component;
	uint8_t length;
};

typedef struct {
	struct CommandComponent components[CMD_MAX_COMPONENTS];
	uint8_t num_components;
} ShellCommand;

void shell_init(UART_HandleTypeDef* uart, void (*terminal)(ShellCommand* cmd, int (*respond)(const char* format, ...)));
void TK_shell(const void *args);
void shell_bridge(int8_t board_id);
void shell_receive_byte(char cbuf, int32_t bridge);
int8_t get_shell_bridge();
bool component_matches(struct CommandComponent* comp, const char* target);

#endif /* APPLICATION_HOSTBOARD_INC_DEBUG_SHELL_H_ */
