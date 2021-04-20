/*
 * board_io.h
 *
 *  Created on: 9 Sep 2020
 *      Author: arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_DEBUG_BOARD_IO_H_
#define APPLICATION_HOSTBOARD_INC_DEBUG_BOARD_IO_H_

#include <stdint.h>
#include <stdbool.h>

#define IO_AUTO      0b10001000100010001000100010001000
#define IO_PIPE      0b01000100010001000100010001000100
#define IO_DIRECT    0b00100010001000100010001000100010
#define __RESERVED0  0b00010001000100010001000100010001

#define IO_CAN		 0b11110000000000001111000000000000
#define IO_TELEMETRY 0b00001111000000000000111100000000
#define __RESERVED1  0b00000000111100000000000011110000
#define __RESERVED2  0b00000000000011110000000000001111

#define IO_INPUT	 0b11111111111111110000000000000000
#define IO_OUTPUT	 0b00000000000000001111111111111111

#ifdef __cplusplus
extern "C" {
#endif

void enable_io_mode(uint32_t mask);
void disable_io_mode(uint32_t mask);
uint32_t get_io_mode();
bool has_io_mode(uint32_t mode);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_HOSTBOARD_INC_DEBUG_BOARD_IO_H_ */
