/*
 * UnbiasedBarometer.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "Sensors/UnbiasedBarometer.h"

#define NUM_BAROMETER_OUTPUTS 2


UnbiasedBarometer::UnbiasedBarometer(const char* identifier, std::initializer_list<Sensor<BarometerData>*> sensors) : UnbiasedSensor(identifier, sensors) {

}

uint16_t UnbiasedBarometer::filterData(BarometerData* measurements, uint8_t count, BarometerData* output) {
	float*** matrix = (float***) pvPortMalloc(NUM_BAROMETER_OUTPUTS * sizeof(float**));
	uint16_t excludedCount = 0;

	for(uint8_t i = 0; i < count; i++) {
		matrix[0][i] = &measurements[i].pressure;
		matrix[1][i] = &measurements[i].temperature;
	}

	for(uint8_t i = 0; i < NUM_BAROMETER_OUTPUTS; i++) {
		excludedCount += removeOutsiders(matrix[i]);
	}

	output->pressure = mean(matrix[0]);
	output->temperature = mean(matrix[1]);

	vPortFree(matrix);

	return excludedCount;
}
