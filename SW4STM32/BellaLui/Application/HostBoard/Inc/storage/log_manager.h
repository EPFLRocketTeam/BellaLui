/*
 * log_manager.h
 *
 *  Created on: 20 Apr 2021
 *      Author: arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_STORAGE_LOG_MANAGER_H_
#define APPLICATION_HOSTBOARD_INC_STORAGE_LOG_MANAGER_H_

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

void on_dump_request();
void on_fullsd_dump_request();
void on_upload_request(uint16_t block_id);


#ifdef __cplusplus
}
#endif

int32_t dump_file_on_sd(const void* filename);
int32_t dump_everything_on_sd(const void* arg);
int32_t upload_block(const void* arg);



#endif /* APPLICATION_HOSTBOARD_INC_STORAGE_LOG_MANAGER_H_ */
