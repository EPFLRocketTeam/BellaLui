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


class AltitudeSimulator : public Sensor<AltitudeData> {
public:
	AltitudeSimulator(const char* identifier);

	bool load();
	bool reset();
	bool fetch(AltitudeData* data);
};



#endif /* APPLICATION_HOSTBOARD_SRC_SENSORS_ALTITUDESIMULATOR_H_ */
