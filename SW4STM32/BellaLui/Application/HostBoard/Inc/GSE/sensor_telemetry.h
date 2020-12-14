/*
 * sensor_telemetry.h
 *
 *  Created on: 22 Oct 2020
 *      Author: lucas
 */

#ifndef APPLICATION_HOSTBOARD_INC_GSE_SENSOR_TELEMETRY_H_
#define APPLICATION_HOSTBOARD_INC_GSE_SENSOR_TELEMETRY_H_

void telemetry_init(void);
void sensors_init(void);
void TK_telemetry_control(void const * argument);
void TK_sensors_control(void const * argument);


#endif /* APPLICATION_HOSTBOARD_INC_GSE_SENSOR_TELEMETRY_H_ */
