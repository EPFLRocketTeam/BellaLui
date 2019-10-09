/*
 * Common.h
 *
 *  Created on: 4 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "datastructs.h"
#include <stm32f4xx_hal.h>

#define CIRC_BUFFER_SIZE 8

extern TIM_HandleTypeDef htim7;

/*
 * States declaration
 */

enum states
{
  STATE_CALIBRATION=0, STATE_IDLE, STATE_LIFTOFF, STATE_COAST, STATE_PRIMARY, STATE_SECONDARY, STATE_TOUCHDOWN
};

volatile uint32_t flight_status;
volatile float32_t airbrakes_angle;
extern volatile float air_speed_state_estimate, altitude_estimate;

volatile uint8_t currentState;
volatile uint32_t LIFTOFF_TIME;

volatile uint32_t currentImuSeqNumber;
volatile uint32_t currentBaroSeqNumber;
volatile uint32_t currentBaroTimestamp;


extern IMU_data IMU_buffer[CIRC_BUFFER_SIZE]; // defined in CAN_handling.c
extern BARO_data BARO_buffer[CIRC_BUFFER_SIZE];

static inline IMU_data* getCurrentIMU_data ()
{
  return &IMU_buffer[currentImuSeqNumber % CIRC_BUFFER_SIZE];
}

static inline BARO_data* getCurrentBARO_data ()
{
  return &BARO_buffer[currentBaroSeqNumber % CIRC_BUFFER_SIZE];
}

static inline void uint8ToFloat (uint8_t* uint8Ptr, float* floatPtr)
{
  uint8_t* floatAsUintPtr = (uint8_t*) floatPtr;
  floatAsUintPtr[0] = uint8Ptr[3];
  floatAsUintPtr[1] = uint8Ptr[2];
  floatAsUintPtr[2] = uint8Ptr[1];
  floatAsUintPtr[3] = uint8Ptr[0];
}

static inline int32_t mod (int32_t x, int32_t n)
{
  int32_t r = x % n;
  return r < 0 ? r + n : r;
}

static inline void floatToUint8 (uint8_t* uint8Ptr, float* floatPtr)
{
  uint8_t* floatAsUintPtr = (uint8_t*) floatPtr;
  uint8Ptr[0] = floatAsUintPtr[3];
  uint8Ptr[1] = floatAsUintPtr[2];
  uint8Ptr[2] = floatAsUintPtr[1];
  uint8Ptr[3] = floatAsUintPtr[0];
}

static inline float32_t abs_fl32 (float32_t v)
{
  return (v >= 0) ? v : -v;
}

static inline float32_t array_mean(float32_t* array, uint8_t arraySize)
{
  uint8_t i;
  float32_t sum = 0.0;

  for(i = 0 ; i < arraySize ; i++)
    {
      sum += array[i];
    }

  return sum/arraySize;
}

#ifdef __cplusplus
 }
#endif

#endif /* INCLUDE_COMMON_H_ */
