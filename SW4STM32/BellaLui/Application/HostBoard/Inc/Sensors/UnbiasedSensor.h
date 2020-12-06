/*
 * UnbiasedSensor.h
 *
 *  Created on: 11 Nov 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_

#include <Sensors/Sensor.h>

#include <stdint.h>


template<class T>
class UnbiasedSensor : public Sensor<T> {
public:
	UnbiasedSensor(const char* identifier, uint8_t count, ...);
	~UnbiasedSensor();
	bool load();
	bool reset();
	bool fetch(T* data);

protected:
	void filterData(T* measurements, uint8_t count, T* output);
	void removeOutsiders(float** data);
	float mean(float** data);

private:
	uint8_t count;
	Sensor<T>** sensors;
	T* measurements;
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_ */
