/*
 * flash_runtime.h
 *
 *  Created on: 27 Nov 2019
 *      Author: TheDreamer979
 */

#ifndef APPLICATION_HOSTBOARD_INC_FLASH_RUNTIME_H_
#define APPLICATION_HOSTBOARD_INC_FLASH_RUNTIME_H_


#include "rocket_fs.h"

#ifdef __cplusplus
extern "C" {
#endif


void init_filesystem();
FileSystem* get_flash_fs();


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_HOSTBOARD_INC_FLASH_RUNTIME_H_ */
