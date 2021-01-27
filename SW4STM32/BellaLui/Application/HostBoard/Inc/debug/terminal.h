/*
 * terminal.h
 *
 *  Created on: 6 Sep 2020
 *      Author: arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_DEBUG_TERMINAL_H_
#define APPLICATION_HOSTBOARD_INC_DEBUG_TERMINAL_H_

#include <stdint.h>

#include "shell.h"

#define SHELL_BRIDGE_CREATE 	0 << 24
#define SHELL_BRIDGE_DESTROY 	1 << 24
#define SHELL_ACK				2 << 24
#define SHELL_ERR				3 << 24

bool is_verbose();
void terminal_execute(ShellCommand* cmd, void (*respond)(const char* format, ...));


#endif /* APPLICATION_HOSTBOARD_INC_DEBUG_TERMINAL_H_ */
