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
#include <stdio.h>


#ifdef __cplusplus
extern "C"
{
#endif

#ifdef DEBUG_MONITOR
#define rocket_log printf
extern void initialise_monitor_handles(void);
#endif

int rocket_log(const char *format, ...);
void rocket_log_init();

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_HOSTBOARD_INC_DEBUG_CONSOLE_H_ */
