/*
 * Common.h
 *
 *  Created on: 4 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#include <misc/datastructs.h>
#include <stm32f4xx_hal.h>

#define CIRC_BUFFER_SIZE 8

#define IGNITION_CODE 9

extern TIM_HandleTypeDef htim7;

/*
 * States declaration
 */


#define NUM_STATES 12


enum State {
	STATE_SLEEP,
	STATE_CALIBRATION,
	STATE_IDLE,
	STATE_OPEN_FILL_VALVE,
	STATE_CLOSE_FILL_VALVE,
	STATE_OPEN_PURGE_VALVE,
	STATE_DISCONNECT_HOSE,
	STATE_LIFTOFF,
	STATE_COAST,
	STATE_PRIMARY,
	STATE_SECONDARY,
	STATE_TOUCHDOWN
};

enum Warning {
	EVENT, WARNING_MOTOR_PRESSURE
};


#ifdef __cplusplus
extern "C" {
#endif

static inline void uint8ToFloat(uint8_t *uint8Ptr, float *floatPtr) {
	uint8_t *floatAsUintPtr = (uint8_t*) floatPtr;
	floatAsUintPtr[0] = uint8Ptr[3];
	floatAsUintPtr[1] = uint8Ptr[2];
	floatAsUintPtr[2] = uint8Ptr[1];
	floatAsUintPtr[3] = uint8Ptr[0];
}

static inline int32_t mod(int32_t x, int32_t n) {
	int32_t r = x % n;
	return r < 0 ? r + n : r;
}

static inline void floatToUint8(uint8_t *uint8Ptr, float *floatPtr) {
	uint8_t *floatAsUintPtr = (uint8_t*) floatPtr;
	uint8Ptr[0] = floatAsUintPtr[3];
	uint8Ptr[1] = floatAsUintPtr[2];
	uint8Ptr[2] = floatAsUintPtr[1];
	uint8Ptr[3] = floatAsUintPtr[0];
}

static inline float abs_fl32(float v) {
	return (v >= 0) ? v : -v;
}

static inline float array_mean(float *array, uint8_t arraySize) {
	uint8_t i;
	float sum = 0.0;

	for (i = 0; i < arraySize; i++) {
		sum += array[i];
	}

	return sum / arraySize;
}

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_COMMON_H_ */
