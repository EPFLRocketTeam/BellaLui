/* telemetry_sending.h
 *
 *  Created on: 10 Jun 2019
 *      Author: Alexandre Devienne
 */

#ifndef TELEMETRY_SENDING_H_
#define TELEMETRY_SENDING_H_

#include <stdbool.h>

#include "misc/datastructs.h"
#include "telemetry/queue/AbstractMessageQueue.h"

#ifdef __cplusplus
extern "C" {
#endif

void registerSendQueue(AbstractMessageQueue<Telemetry_Message> *queue);

bool telemetrySendGPS(uint32_t timestamp, GPS_data data);
bool telemetrySendIMU(uint32_t timestamp, IMU_data data);
bool telemetrySendBaro(uint32_t timestamp, BARO_data data);
bool telemetrySendState(uint32_t timestamp, bool id, float value, uint8_t av_state);
bool telemetrySendMotorPressure(uint32_t timestamp, uint32_t pressure);
bool telemetrySendAirbrakesAngle(uint32_t timestamp, int32_t angle);
bool telemetrySendPropulsionData(uint32_t timestamp, PropulsionData* payload);

#ifdef __cplusplus
}
#endif


#endif /* TELEMETRY_SENDING_H_ */
