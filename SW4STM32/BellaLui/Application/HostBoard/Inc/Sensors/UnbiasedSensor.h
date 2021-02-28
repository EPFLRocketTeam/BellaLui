/*
 * UnbiasedSensor.h
 *
 *  Created on: 11 Nov 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_

#include <Sensors/Sensor.h>
#include <Embedded/system.h>

#include <stdint.h>
#include <initializer_list>



template<class T>
class UnbiasedSensor : public Sensor<T> {
public:
	UnbiasedSensor(const char* identifier, std::initializer_list<Sensor<T>*> sensors);
	~UnbiasedSensor();
	bool load();
	bool unload();
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


template<class T>
UnbiasedSensor<T>::UnbiasedSensor(const char* identifier, std::initializer_list<Sensor<T>*> sensors) : Sensor<T>(identifier) {
	this->count = 0;
	this->excludedCount = 0;

	this->sensors = (Sensor<T>**) pvPortMalloc(sensors.size() * sizeof(Sensor<T>*));
	this->measurements = (T*) pvPortMalloc(sensors.size() * sizeof(T));

	for(Sensor<T>* sensor : sensors) {
		this->sensors[count++] = sensor;
	}
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
bool UnbiasedSensor<T>::unload() {
	bool status = false;

	for(uint8_t i = 0; i < count; i++) {
		status |= this->sensors[i]->unload();
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



#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_ */
