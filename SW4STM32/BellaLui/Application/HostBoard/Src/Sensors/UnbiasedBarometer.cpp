/*
 * UnbiasedBarometer.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "Sensors/UnbiasedBarometer.h"

#define NUM_BAROMETER_OUTPUTS 2


void UnbiasedBarometer::filterData(BarometerData* measurements, uint8_t count, BarometerData* output) {
	float*** matrix = (float***) pvPortMalloc(NUM_BAROMETER_OUTPUTS * sizeof(float**));

	for(uint8_t i = 0; i < count; i++) {
		matrix[0][i] = &measurements[i].pressure;
		matrix[1][i] = &measurements[i].temperature;
	}

	for(uint8_t i = 0; i < NUM_BAROMETER_OUTPUTS; i++) {
		removeOutsiders(matrix[i]);
	}

	output->pressure = mean(matrix[0]);
	output->temperature = mean(matrix[1]);

	vPortFree(matrix);
}
