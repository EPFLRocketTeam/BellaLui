/*
 * console.h
 *
 *  Created on: 11 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_DEBUG_CONSOLE_H_
#define APPLICATION_HOSTBOARD_INC_DEBUG_CONSOLE_H_

// Semi hosting has to be enabled in eclipse, otherwise the program will sigtrap at the instruction initialise_monitor_handler() in main.c

#include <threads.h>

#ifdef DEBUG
#define rocket_log printf
int rocket_log(const char *format, ...);
extern void initialise_monitor_handles(void);
void rocket_log_init();
#endif

#endif /* APPLICATION_HOSTBOARD_INC_DEBUG_CONSOLE_H_ */
