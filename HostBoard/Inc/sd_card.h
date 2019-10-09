/*
 * data_handling.h
 *
 *  Created on: 19 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#include "cmsis_os.h"

#ifndef SD_SYNC_H_
#define SD_SYNC_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
  {
#endif

void TK_sd_sync (const void* args);

osStatus initSdFile ();

int sd_write(char str[], int size);

#ifdef __cplusplus
}
#endif

#endif //SD_SYNC_H_
