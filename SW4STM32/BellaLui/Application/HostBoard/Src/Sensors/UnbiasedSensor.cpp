/*
 * UnbiasedSensor.cpp
 *
 *  Created on: 11 Nov 2020
 *      Author: Arion
 */

#include <Sensors/UnbiasedSensor.h>

#include <cstdarg>
#include <cmsis_os.h>

template<class T>
UnbiasedSensor<T>::UnbiasedSensor(const char* identifier, uint8_t count, ...) : Sensor<T>(identifier), count(count) {
	va_list args;

	this->sensors = (Sensor<T>**) pvPortMalloc(sensors * sizeof(Sensor<T>*));
	this->measurements = (T*) pvPortMalloc(count * sizeof(T));

	va_start(args, sensors);

	for(uint8_t i = 0; i < count; i++) {
		this->sensors[i] = va_arg(args, Sensor<T>*);
	}

	va_end(args);
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

	filterData(measurements, count, data);

	return status;
}

template<class T>
void UnbiasedSensor<T>::removeOutsiders(float** data) {
	// TODO
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
