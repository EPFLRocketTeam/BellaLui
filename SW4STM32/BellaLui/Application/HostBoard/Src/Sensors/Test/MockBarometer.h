/*
 * Barometer.h
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_MOCKBAROMETER_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_MOCKBAROMETER_H_


#include <Sensors/Sensor.h>

#include <cstdint>

class MockBarometer : public Sensor<BarometerData> {
public:
	MockBarometer(const char* identifier);
	MockBarometer(const char* identifier, uint32_t start, uint32_t end);
	~MockBarometer() { unload(); }

	bool load();
	bool unload();
	bool fetch(BarometerData* data);

private:
	uint32_t start;
	uint32_t end;
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_MOCKBAROMETER_H_ */
