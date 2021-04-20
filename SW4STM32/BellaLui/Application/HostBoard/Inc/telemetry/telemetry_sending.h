/* telemetry_sending.h
 *
 *  Created on: 10 Jun 2019
 *      Author: Alexandre Devienne
 */

#ifndef TELEMETRY_SENDING_H_
#define TELEMETRY_SENDING_H_

#include <stdbool.h>

#include "misc/datastructs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TELE_TIMEMIN 200
#define GPS_TIMEMIN 200
#define STATE_TIMEMIN 200
#define PROP_DATA_TIMEMIN 100
#define AB_TIMEMIN 100
//#define TELE_RAW_TIMEMIN 100

bool telemetrySendGPS(uint32_t timestamp, GPS_data data);
bool telemetrySendIMU(uint32_t timestamp, IMU_data data);
bool telemetrySendBaro(uint32_t timestamp, BARO_data data);
bool telemetrySendState(uint32_t timestamp, uint8_t id, float value, uint8_t av_state); // TODO: uint8_t or bool ???
bool telemetrySendMotorPressure(uint32_t timestamp, uint32_t pressure);
bool telemetrySendAirbrakesAngle(uint32_t timestamp, float angle); // TODO: int32_t or float ???
bool telemetrySendPropulsionData(uint32_t timestamp, PropulsionData* payload);

#ifdef __cplusplus
}
#endif


#endif /* TELEMETRY_SENDING_H_ */
