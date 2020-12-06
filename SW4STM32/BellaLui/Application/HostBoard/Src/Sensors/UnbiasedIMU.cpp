/*
 * UnbiasedIMU.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "Sensors/UnbiasedIMU.h"

#include <cmsis_os.h>

#define NUM_IMU_OUTPUTS 6

void UnbiasedIMU::filterData(IMUData* measurements, uint8_t count, IMUData* output) {
	float*** matrix = (float***) pvPortMalloc(NUM_IMU_OUTPUTS * sizeof(float**));

	for(uint8_t i = 0; i < count; i++) {
		matrix[0][i] = &measurements[i].accel.x;
		matrix[1][i] = &measurements[i].accel.y;
		matrix[2][i] = &measurements[i].accel.z;
		matrix[3][i] = &measurements[i].gyro.x;
		matrix[4][i] = &measurements[i].gyro.y;
		matrix[5][i] = &measurements[i].gyro.z;
	}

	for(uint8_t i = 0; i < NUM_IMU_OUTPUTS; i++) {
		removeOutsiders(matrix[i]);
	}

	output->accel.x = mean(matrix[0]);
	output->accel.y = mean(matrix[1]);
	output->accel.z = mean(matrix[2]);
	output->gyro.x = mean(matrix[3]);
	output->gyro.y = mean(matrix[4]);
	output->gyro.z = mean(matrix[5]);

	vPortFree(matrix);
}
