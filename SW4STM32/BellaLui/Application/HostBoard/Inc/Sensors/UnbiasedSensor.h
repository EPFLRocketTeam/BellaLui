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
#include <initializer_list>



template<class T>
class UnbiasedSensor : public Sensor<T> {
public:
	UnbiasedSensor(const char* identifier, std::initializer_list<Sensor<T>*> sensors);
	~UnbiasedSensor();
	bool load();
	bool reset();
	bool fetch(T* data);
	uint16_t getExcludedCount() { return excludedCount; }

protected:
	virtual uint16_t filterData(T* measurements, uint8_t count, T* output) = 0;
	uint8_t removeOutsiders(float** data);
	float mean(float** data);

private:
	uint8_t count;
	Sensor<T>** sensors;
	T* measurements;
	uint16_t excludedCount;
};





#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_ */
