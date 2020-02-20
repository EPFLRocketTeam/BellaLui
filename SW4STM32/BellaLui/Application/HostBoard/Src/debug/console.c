/*
 * console.c
 *
 *  Created on: Feb 17, 2020
 *      Author: Quentin
 */

#include <debug/console.h>


int rocket_log(const char *format, ...) {
	return 0;
}

void rocket_log_init() {
	#ifdef DEBUG
	initialise_monitor_handles();
	#endif
}
