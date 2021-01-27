/*
 * board_io.c
 *
 *  Created on: 9 Sep 2020
 *      Author: arion
 */


#include "debug/board_io.h"

static volatile uint32_t io_mode = (IO_INPUT & IO_AUTO) | (IO_OUTPUT & IO_AUTO);

void enable_io_mode(uint32_t mode) {
	io_mode |= mode;
}

void disable_io_mode(uint32_t mode) {
	io_mode &= ~mode;
}

uint32_t get_io_mode() {
	return io_mode;
}

bool has_io_mode(uint32_t mode) {
	return (io_mode & mode) != 0;
}
