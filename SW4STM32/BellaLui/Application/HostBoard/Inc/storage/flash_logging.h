/*
 * flash_logging.h
 *
 *  Created on: 22 Oct 2019
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_
#define APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_


#include <can_transmission.h>
#include "rocket_fs.h"
#include "flash.h"


// Bigger buffer does not work
#define LOGGING_BUFFER_SIZE (8 * 64)

void init_logging();
void start_logging();
void stop_logging();
void flash_log(CAN_msg message);
void TK_logging_thread(void const *pvArgs);

void acquire_flash_lock();
void release_flash_lock();

#endif /* APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_ */
