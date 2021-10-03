/*
 * state_manager.h
 *
 *  Created on: 28 Apr 2021
 *      Author: arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_MISC_STATE_MANAGER_H_
#define APPLICATION_HOSTBOARD_INC_MISC_STATE_MANAGER_H_


#include "misc/common.h"


#ifdef __cplusplus
extern "C" {
#endif


void TK_state_machine(void const* argument);

void onIMUDataReception(IMU_data data);
void onBarometerDataReception(BARO_data data);
void onStateAcknowledged(enum State newState);

enum State getAvionicsState();
int32_t getLiftoffTime();


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_HOSTBOARD_INC_MISC_STATE_MANAGER_H_ */
