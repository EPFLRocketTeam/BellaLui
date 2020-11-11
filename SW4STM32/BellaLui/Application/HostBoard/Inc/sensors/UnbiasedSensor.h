/*
 * UnbiasedSensor.h
 *
 *  Created on: 11 Nov 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_

#include "sensors/Sensor.h"


template<class T>
class UnbiasedSensor : public Sensor<T> {
public:
	UnbiasedSensor() : Sensor<T>("Sensor bias filter") {}
	bool load();
	bool reset();
	bool fetch(T* data);
private:
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_ */
