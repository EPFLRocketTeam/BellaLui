/*
 * AltitudeSimulator.h
 *
 *  Created on: 22 Jan 2021
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_SRC_SENSORS_ALTITUDESIMULATOR_H_
#define APPLICATION_HOSTBOARD_SRC_SENSORS_ALTITUDESIMULATOR_H_

#include "Sensors/Sensor.h"
#include "Sensors/DataStructures.h"

#include <iostream>

class AltitudeSimulator : public Sensor<AltitudeData> {
public:
	AltitudeSimulator(const char* identifier, uint32_t start, uint32_t end);
	~AltitudeSimulator() { unload(); }

	bool load();
	bool unload();
	bool fetch(AltitudeData* data);

private:
	FILE* file;
	uint32_t start;
	uint32_t end;
};



#endif /* APPLICATION_HOSTBOARD_SRC_SENSORS_ALTITUDESIMULATOR_H_ */
