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
bool telemetry_handleWarningPacketData(bool id, float value, uint8_t av_state);
bool telemetry_handleABData();
bool telemetry_handleIgnitionPacket(uint8_t* rxPacketBuffer);
bool telemetry_handleOrderPacket(uint8_t* rxPacketBuffer);



#endif /* TELEMETRY_HANDLING_H_ */
