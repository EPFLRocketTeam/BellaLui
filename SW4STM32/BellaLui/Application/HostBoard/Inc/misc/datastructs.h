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

typedef struct
{
  float x, y, z;
} float3D;

typedef struct
{
  float3D acceleration;
  float3D eulerAngles;
} IMU_data;

typedef struct
{
  float temperature;
  float pressure;
  float altitude;
  float base_pressure;
  float base_altitude;
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
	uint32_t thrust_cmd;
	uint32_t tvc_status;
} TVCStatus;

typedef struct
{
  uint16_t size;
  uint8_t buf[];
} Telemetry_Message;

typedef struct
{
  float hdop; // m
  float lat; // deg
  float lon; // deg
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
