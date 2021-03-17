/*
 * Sensor.h
 *
 *  Created on: 11 Nov 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_SENSOR_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_SENSOR_H_

#include <stdbool.h>

#include <Sensors/DataStructures.h>


template<class T>
class Sensor {
public:
	Sensor(const char* identifier) : identifier(identifier) {}
	virtual ~Sensor() {};
	virtual bool load() = 0;
	virtual bool unload() = 0;
	virtual bool fetch(T* data) = 0;
	bool reset() { return unload() && load(); }
	const char* name() { return identifier; }
private:
	const char* identifier;
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_SENSOR_H_ */
