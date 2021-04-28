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
#include <Embedded/system.h>

#include <algorithm>
#include <stdint.h>
#include <initializer_list>
#include <iostream>

#include <cmath>



template<class T>
class UnbiasedSensor : public Sensor<T> {
public:
	UnbiasedSensor(const char* identifier, std::initializer_list<Sensor<T>*> sensors);
	~UnbiasedSensor();
	bool load();
	bool unload();
	bool fetch(T* data);
	uint16_t getExcludedCount() { return excludedCount; }
	uint8_t getCount() { return count; }

protected:
	virtual uint16_t filterData(T* measurements, uint8_t count, T* output) = 0;
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

	if(status)
		this->ready = true;

	return status;
}

template<class T>
bool UnbiasedSensor<T>::unload() {
	bool status = false;

	for(uint8_t i = 0; i < count; i++) {
		status |= this->sensors[i]->unload();
	}

	if(status)
		this->ready = false;

	return status;
}

template<class T>
bool UnbiasedSensor<T>::fetch(T* data) {
	if(!this->ready)
		return false;

	bool status = false;

	for(uint8_t i = 0; i < count; i++) {
		status |= this->sensors[i]->fetch(&measurements[i]);
	}

	this->excludedCount = filterData(measurements, count, data);

	return status;
}

template<class T>
uint8_t UnbiasedSensor<T>::removeOutsiders(float** data) {
	uint8_t sorting_array[SENSORS_NB] = {0,1,2,3}; //TODO:magic numbers? could work with array of pointers of data instead of indexes... faster code I believe but more RAM (insignificant?)
	sortingNetwork(data, sorting_array);

	return filterOutData(data,sorting_array,0,3);
}

template<class T>
void UnbiasedSensor<T>::sortingNetwork(float** data, uint8_t* sorting_array) {
	std::sort(sorting_array, sorting_array + count, [data](uint8_t a, uint8_t b) {
		if(nullptr == data[a])
			return false;
		if(nullptr == data[b])
			return true;
		return *data[a] < *data[b];
	});
}

template<class T>
void UnbiasedSensor<T>::standardDeviationMean(float** data, float& mean, float& st_dev) {
	mean = 0.0f;
	uint8_t num = 0;

	for(uint8_t i = 0; i < SENSORS_NB; ++i){
		if(data[i] != nullptr) {
			mean += *data[i];
			num++;
		}
	}
	mean /= num;

	float variance = 0.0f;
	for(uint8_t i = 0; i < SENSORS_NB; ++i){
		if(data[i] != nullptr) {
			variance += (*data[i]-mean)*(*data[i]-mean);
		}
	}
	variance /= num;

	st_dev = sqrt(variance);
	return;
}

template<class T>
uint8_t UnbiasedSensor<T>::filterOutData(float** data, uint8_t* sort_array, uint8_t index_min, uint8_t index_max){ //TODO:recursive function: bad for heap?
	float test_value = 0.0f;
	if(index_max-index_min == SENSORS_NB-1){
		test_value = (*data[sort_array[index_min+1]] + *data[sort_array[index_max-1]])/2;
	}else if(index_max-index_min == SENSORS_NB-2){
		test_value = *data[sort_array[index_min+1]];
	}else{
		return 0; //stops recursive loop
	}

	float mean = 0.0f;
	float st_dev = 0.0f;
	standardDeviationMean(data, mean, st_dev);

	if(test_value >= mean - 0.3*st_dev && test_value <= mean + 0.3*st_dev){
		return 0;
	}else{
		if( abs(*data[sort_array[index_min]]-test_value) > abs(*data[sort_array[index_max]]-test_value) ){
			data[sort_array[index_min]] = nullptr;
			return 1 + filterOutData(data, sort_array, index_min + 1, index_max);
		}else{
			data[sort_array[index_max]] = nullptr;
			return 1 + filterOutData(data, sort_array, index_min, index_max - 1);
		}
	}

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
