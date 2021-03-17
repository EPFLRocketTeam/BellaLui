/*
 * IMU.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "MockIMU.h"

#define ACCEL_ACQUISITION_DIVIDER 1
#define GYRO_ACQUISITION_DIVIDER 10


MockIMU::MockIMU(const char* identifier) : MockIMU(identifier, 0, 100) {

}

MockIMU::MockIMU(const char* identifier, uint32_t start, uint32_t end) : Sensor<IMUData>(identifier), file(nullptr), start(start), end(end) {

}

bool MockIMU::load() {
	file = fopen(name(), "r");
	return file != nullptr;
}

bool MockIMU::unload() {
	return file != nullptr && fclose(file) == 0;
}

bool MockIMU::fetch(IMUData* data) {
	float time_read;

	do {
		fscanf(file, "%f,%f,%f,%f,%f,%f,%f", &time_read, &data->accel.x, &data->accel.y, &data->accel.z, &data->gyro.x, &data->gyro.y, &data->gyro.z);
	} while((uint32_t) time_read < start);

	if((uint32_t) time_read >= end) {
		return false;
	}

	return true;
}
