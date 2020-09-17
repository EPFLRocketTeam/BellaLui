/*
 * sensor_telemetry.h
 *
 *  Created on: 8 Jul 2020
 *      Author: lucaspallez
 */

#ifndef SENSOR_TELEMETRY_H_
#define SENSOR_TELEMETRY_H_

void telemetry_init(void);
void sensors_init(void);

void TK_telemetry_control(void const * argument);
void TK_sensors_control(void const * argument);

float read_tank_temp(void);

#endif /* SENSOR_TELEMETRY_H_ */
