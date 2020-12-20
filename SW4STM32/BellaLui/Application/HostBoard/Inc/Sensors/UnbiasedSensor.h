/*
 * UnbiasedSensor.h
 *
 *  Created on: 11 Nov 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_

#define SENSORS_NB 4

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
	uint16_t filterData(T* measurements, uint8_t count, T* output);
	uint8_t removeOutsiders(float** data);
	float mean(float** data);

private:
	uint8_t count;
	Sensor<T>** sensors;
	T* measurements;
	uint16_t excludedCount;
	void sortingNetwork(float** data, uint8_t* sorting_array);
	void standardDeviationMean(float** data, float& mean, float& st_dev);
	uint8_t filterOutData(float** data, uint8_t* sort_array, uint8_t index_min, uint8_t index_max);

};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_ */
