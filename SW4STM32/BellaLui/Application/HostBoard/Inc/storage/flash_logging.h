/*
 * flash_logging.h
 *
 *  Created on: 22 Oct 2019
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_
#define APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_


#include "CAN_communication.h"



// Bigger buffer does not work
#define LOGGING_BUFFER_SIZE (8 * 8)

void init_logging();
void flash_log(CAN_msg message);
void TK_logging_thread(void const *pvArgs);

void acquire_flash_lock();
void release_flash_lock();

void on_dump_request();
void on_fullsd_dump_request();
int32_t dump_file_on_sd(const char* filename);
int32_t dump_everything_on_sd(void* arg);

#endif /* APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_ */
