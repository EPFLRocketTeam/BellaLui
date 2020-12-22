/*
 * UnbiasedSensor.cpp
 *
 *  Created on: 11 Nov 2020
 *      Author: Arion
 */

#include <Embedded/system.h>
#include <Sensors/UnbiasedSensor.h>


template<class T>
UnbiasedSensor<T>::UnbiasedSensor(const char* identifier, std::initializer_list<Sensor<T>*> sensors) : Sensor<T>(identifier) {
	this->count = 0;
	this->excludedCount = 0;

	for(Sensor<T>* sensor : sensors) {
		this->sensors[count++] = sensor;
	}

	this->sensors = (Sensor<T>**) pvPortMalloc(sensors * sizeof(Sensor<T>*));
	this->measurements = (T*) pvPortMalloc(count * sizeof(T));
}

template<class T>
UnbiasedSensor<T>::~UnbiasedSensor() {
	vPortFree(sensors);
	vPortFree(measurements);
}

template<class T>
bool UnbiasedSensor<T>::load() {
	bool status = false;

	for(uint8_t i = 0; i < count; i++) {
		status |= this->sensors[i]->load();
	}

	return status;
}

template<class T>
bool UnbiasedSensor<T>::reset() {
	bool status = false;

	for(uint8_t i = 0; i < count; i++) {
		status |= this->sensors[i]->reset();
	}

	return status;
}

template<class T>
bool UnbiasedSensor<T>::fetch(T* data) {
	bool status = false;

	for(uint8_t i = 0; i < count; i++) {
		status |= this->sensors[i]->fetch(&measurements[i]);
	}

	this->excludedCount = filterData(measurements, count, data);

	return status;
}

template<class T>
uint8_t UnbiasedSensor<T>::removeOutsiders(float** data) {
	return 0;
}

template<class T>
float UnbiasedSensor<T>::mean(float** data) {
	float result = 0.0f;
	uint8_t num = 0;

	for(uint8_t i = 0; i < count; i++) {
		if(data[i] != nullptr) {
			result += *data[i];
			num++;
		}
	}

	return result / num;
}
