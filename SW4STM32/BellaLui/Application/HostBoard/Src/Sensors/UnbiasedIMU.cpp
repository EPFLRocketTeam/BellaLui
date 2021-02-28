/*
 * UnbiasedIMU.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "Sensors/UnbiasedIMU.h"

#include "Embedded/system.h"


#define NUM_IMU_OUTPUTS 6

UnbiasedIMU::UnbiasedIMU(const char* identifier, std::initializer_list<Sensor<IMUData>*> sensors) : UnbiasedSensor(identifier, sensors) {

}

uint16_t UnbiasedIMU::filterData(IMUData* measurements, uint8_t count, IMUData* output) {
	float** matrix = (float**) pvPortMalloc(NUM_IMU_OUTPUTS * count * sizeof(float*));
	uint16_t excludedCount = 0;


	for(uint8_t i = 0; i < count; i++) {
		matrix[0 * count + i] = &measurements[i].accel.x;
		matrix[1 * count + i] = &measurements[i].accel.y;
		matrix[2 * count + i] = &measurements[i].accel.z;
		matrix[3 * count + i] = &measurements[i].gyro.x;
		matrix[4 * count + i] = &measurements[i].gyro.y;
		matrix[5 * count + i] = &measurements[i].gyro.z;
	}


	for(uint8_t i = 0; i < NUM_IMU_OUTPUTS; i++) {
		excludedCount += removeOutsiders(&matrix[i * count]);
	}

	output->accel.x = mean(&matrix[0 * count]);
	output->accel.y = mean(&matrix[1 * count]);
	output->accel.z = mean(&matrix[2 * count]);
	output->gyro.x = mean(&matrix[3 * count]);
	output->gyro.y = mean(&matrix[4 * count]);
	output->gyro.z = mean(&matrix[5 * count]);

	vPortFree(matrix);

	return excludedCount;
}
