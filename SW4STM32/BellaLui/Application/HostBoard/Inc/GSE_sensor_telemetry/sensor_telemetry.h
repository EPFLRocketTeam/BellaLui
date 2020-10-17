/*
 * sensor_telemetry.h
 *
 *  Created on: 8 Jul 2020
 *      Author: lucaspallez
 */

#ifndef SENSOR_TELEMETRY_H_
#define SENSOR_TELEMETRY_H_

#define SAMPLING_TIME 	20 //ms
#define ADC_FREQ		3200 //Hz
#define NB_SAMPLES		64
#define NB_SENSOR		4

#define SEND_RATE		20

typedef struct {
	int32_t tank_temp;
	uint32_t hose_pressure;
	int32_t hose_temp;
	uint32_t vbat;
	uint32_t time;
}SENSOR_DATA_t;

typedef struct {
	int32_t tank_temp;
	int32_t hose_pressure;
	int32_t hose_temp;
	uint32_t vbat;
}SAMPLING_DATA_t;

void telemetry_init(void);
void sensors_init(void);

void TK_telemetry_control(void const * argument);
void TK_sensors_control(void const * argument);

SENSOR_DATA_t sensor_get_data_struct(void);
uint32_t sensor_get_time(void);

#endif /* SENSOR_TELEMETRY_H_ */
