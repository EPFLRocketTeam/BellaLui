/*
 * log_manager.h
 *
 *  Created on: 20 Apr 2021
 *      Author: arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_STORAGE_LOG_MANAGER_H_
#define APPLICATION_HOSTBOARD_INC_STORAGE_LOG_MANAGER_H_

#include <stdint.h>

void on_dump_request();
void on_fullsd_dump_request();
void on_upload_request(uint16_t block_id);
int32_t dump_file_on_sd(void* filename);
int32_t dump_everything_on_sd(void* arg);
int32_t upload_block(void* arg);


#endif /* APPLICATION_HOSTBOARD_INC_STORAGE_LOG_MANAGER_H_ */
