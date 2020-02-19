/* telemetry_handling.h
 *
 *  Created on: 10 Jun 2019
 *      Author: Alexandre Devienne
 */

#ifndef TELEMETRY_HANDLING_H_
#define TELEMETRY_HANDLING_H_

#include <stdbool.h>

#include "../../../HostBoard/Inc/Misc/datastructs.h"

bool telemetry_handleGPSData(GPS_data data);
bool telemetry_handleIMUData(IMU_data data);
bool telemetry_handleBaroData(BARO_data data);
bool telemetry_handleMotorPressureData(uint32_t pressure);
bool telemetry_handleWarningPacketData(bool id, uint32_t value);
bool telemetry_handleABData();
//bool telemetry_handleTelemetryRaw(float32_t euler, float32_t accelerometer, float32_t temp, float32_t pressure);



#endif /* TELEMETRY_HANDLING_H_ */
