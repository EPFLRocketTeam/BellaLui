/*
 * IMU.h
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_MOCKIMU_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_MOCKIMU_H_


#include <Sensors/Sensor.h>

#include <cstdint>
#include <iostream>


class MockIMU : public Sensor<IMUData> {
public:
	MockIMU(const char* identifier);
	MockIMU(const char* identifier, uint32_t start, uint32_t end);
	~MockIMU() { unload(); }

	bool load();
	bool unload();
	bool fetch(IMUData* data);

private:
	FILE* file;
	uint32_t start;
	uint32_t end;
};

#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_MOCKIMU_H_ */
