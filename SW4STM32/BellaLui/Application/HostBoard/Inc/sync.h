/*
 * sync.h
 *
 *  Created on: 16 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SYNC_H_
#define APPLICATION_HOSTBOARD_INC_SYNC_H_

#include <stdint.h>
#include <stdbool.h>

void sync_logic(uint16_t period);
bool do_privileged_io();
bool end_privileged_io();

#endif /* APPLICATION_HOSTBOARD_INC_SYNC_H_ */
