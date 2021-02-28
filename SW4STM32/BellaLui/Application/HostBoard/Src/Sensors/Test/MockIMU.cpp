/*
 * IMU.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "MockIMU.h"

#define ACCEL_ACQUISITION_DIVIDER 1
#define GYRO_ACQUISITION_DIVIDER 10


MockIMU::MockIMU(const char* identifier) : Sensor(identifier) {

}


bool MockIMU::load() {
	return true;
}

bool MockIMU::unload() {
	return true;
}

bool MockIMU::fetch(IMUData* data) {
	return true;
}
