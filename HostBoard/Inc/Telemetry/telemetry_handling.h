/*
 * telemetry_handling.h
 *
 *  Created on: 10 Jun 2019
 *      Author: Alexandre Devienne
 */

#ifndef TELEMETRY_HANDLING_H_
#define TELEMETRY_HANDLING_H_

#include <stdbool.h>
#include "Misc/datastructs.h"

bool telemetry_handleGPSData(GPS_data data);
bool telemetry_handleIMUData(IMU_data data);
bool telemetry_handleBaroData(BARO_data data);

#endif /* TELEMETRY_HANDLING_H_ */
