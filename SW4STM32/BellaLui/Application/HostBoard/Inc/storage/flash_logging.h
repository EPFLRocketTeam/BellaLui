/*
 * flash_logging.h
 *
 *  Created on: 22 Oct 2019
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_
#define APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_


#include <can_transmission.h>



// Bigger buffer does not work
#define LOGGING_BUFFER_SIZE (8 * 64)

void init_logging();
void start_logging();
void stop_logging();
void flash_log(CAN_msg message);
void TK_logging_thread(void const *pvArgs);

void acquire_flash_lock();
void release_flash_lock();

void on_dump_request();
void on_fullsd_dump_request();
void on_upload_request(uint16_t block_id);
int32_t dump_file_on_sd(void* filename);
int32_t dump_everything_on_sd(void* arg);
int32_t upload_block(void* arg);

#endif /* APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_ */
