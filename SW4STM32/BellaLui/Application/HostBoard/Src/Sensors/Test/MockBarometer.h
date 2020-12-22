/*
 * Barometer.h
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_MOCKBAROMETER_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_MOCKBAROMETER_H_


#include <Sensors/Sensor.h>

class MockBarometer : public Sensor<BarometerData> {
public:
	MockBarometer(const char* identifier);

	bool load();
	bool reset();
	bool fetch(BarometerData* data);
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_MOCKBAROMETER_H_ */
