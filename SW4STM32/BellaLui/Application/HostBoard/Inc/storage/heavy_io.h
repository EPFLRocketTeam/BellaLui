/*
 * heavy_io.h
 *
 *  Created on: 30 Oct 2019
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_HEAVY_IO_H_
#define APPLICATION_HOSTBOARD_INC_HEAVY_IO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_heavy_scheduler();
void schedule_heavy_task(int32_t (*task)(const void*), const void* arg, void (*feedback)(int32_t));
void TK_heavy_io_scheduler();

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_HOSTBOARD_INC_HEAVY_IO_H_ */
