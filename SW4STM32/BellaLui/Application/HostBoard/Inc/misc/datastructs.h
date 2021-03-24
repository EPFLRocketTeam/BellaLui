/*
 * datastructs.h
 *
 *  Created on: 5 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef INCLUDE_DATASTRUCTS_H_
#define INCLUDE_DATASTRUCTS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

typedef float float32_t;
typedef double float64_t;

typedef struct
{
  float32_t x, y, z;
} float3D;

typedef struct
{
  float3D acceleration;
  float3D eulerAngles;
} IMU_data;

typedef struct
{
  float32_t temperature;
  float32_t pressure;
  float32_t altitude;
  float32_t base_pressure;
  float32_t base_altitude;
} BARO_data;

typedef struct
{
  int32_t pressure1;
  int32_t pressure2;
  int16_t temperature1;
  int16_t temperature2;
  int16_t temperature3;
  uint32_t status;
  int32_t motor_position;
} PropulsionData;

typedef struct
{
  uint16_t size;
  uint8_t buf[];
} Telemetry_Message;

typedef struct
{
  float32_t hdop; // m
  float32_t lat; // deg
  float32_t lon; // deg
  int32_t altitude; // cm
  uint8_t sats;
} GPS_data;


typedef struct
{
  void* ptr;
  uint16_t size;
} String_Message;


#ifdef __cplusplus
 }
#endif

#endif /* INCLUDE_DATASTRUCTS_H_ */
