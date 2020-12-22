/*
 * IMU.h
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_MOCKIMU_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_MOCKIMU_H_


#include <Sensors/Sensor.h>


class MockIMU : public Sensor<IMUData> {
public:
	MockIMU(const char* identifier);

	bool load();
	bool reset();
	bool fetch(IMUData* data);
};

#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_MOCKIMU_H_ */
