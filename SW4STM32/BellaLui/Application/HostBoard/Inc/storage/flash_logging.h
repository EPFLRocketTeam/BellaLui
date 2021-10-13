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

#ifdef __cplusplus
extern "C" {
#endif


void init_logging();
void start_logging();
void stop_logging();
void enable_flushing();
void disable_flushing();
void flash_log(CAN_msg message);
void TK_logging_thread(void const *pvArgs);

void acquire_flash_lock();
void release_flash_lock();

#ifdef __cplusplus
}
#endif


#endif /* APPLICATION_HOSTBOARD_INC_FLASH_LOGGING_H_ */
