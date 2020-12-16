/*
 * UnbiasedSensor.cpp
 *
 *  Created on: 11 Nov 2020
 *      Author: Arion
 */

#include <Sensors/UnbiasedSensor.h>

#include <cmsis_os.h>

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
	uint8_t sorting_array[4] = {0,1,2,3}; //TODO:magic number:4 = #sensors
	sorting_network(data, sorting_array);
	float st_dev = standard_deviation(data);
	float mean = mean(data);
	if(IS_IN_INTERVAL((*data[sorting_array[1]]+*data[sorting_array[2]])/2, mean-0.3*st_dev, mean+0.3*st_dev)) {//TODO:assuming "count" is 4 + magic number 0.3 = conf interval
		return 0;
	}else{
		switch(THE_FURTHER_INDEX(data, sorting_array, 0, 3, (*data[sorting_array[1]]+*data[sorting_array[2]])/2)){
		case 0:{
			data[sorting_array[0]] = nullptr;
			mean = mean(data);
			st_dev = standard_deviation(data);
			if(IS_IN_INTERVAL(*data[sorting_array[2]], mean-0.3*st_dev, mean+0.3*st_dev)) {
				return 1;
			}else{
				switch(THE_FURTHER_INDEX(data, sorting_array, 1, 3, *data[sorting_array[2]])){
				case 1:{
					data[sorting_array[1]] = nullptr;
					return 2;
				}
				case 3:{
					data[sorting_array[3]] = nullptr;
					return 2;
				}
				default: return 1;
				}
			}
			break;
		}
		case 3:{
			data[sorting_array[3]] = nullptr;
			mean = mean(data);
			st_dev = standard_deviation(data);
			if(IS_IN_INTERVAL(*data[sorting_array[1]], mean-0.3*st_dev, mean+0.3*st_dev)) {
				return 1;
			}else{
				switch(THE_FURTHER_INDEX(data, sorting_array, 0, 2, *data[sorting_array[1]])){
				case 0:{
					data[sorting_array[0]] = nullptr;
					return 2;
				}
				case 2:{
					data[sorting_array[2]] = nullptr;
					return 2;
				}
				default: return 1;
				}
			}
			break;
		}
		default: return 0;
		}

	}
}

template<class T>
void UnbiasedSensor<T>::sorting_network(float** data, uint8_t* sorting_array) { //TODO: can use sorting network? fixed amount of sensors?
    if(*data[sorting_array[0]] > *data[sorting_array[2]])		swap(sorting_array[0],sorting_array[2]);
    if(*data[sorting_array[1]] > *data[sorting_array[3]]) 		swap(sorting_array[1],sorting_array[3]);
    if(*data[sorting_array[0]] > *data[sorting_array[1]])		swap(sorting_array[0],sorting_array[1]);
    if(*data[sorting_array[2]] > *data[sorting_array[3]]) 		swap(sorting_array[2],sorting_array[3]);
    if(*data[sorting_array[1]] > *data[sorting_array[2]])		swap(sorting_array[1],sorting_array[2]);
}

template<class T>
float UnbiasedSensor<T>::standard_deviation(float** data) {
	float mean = mean(data); //TODO: assuming count is = 4
	uint8_t num = 0;

	float variance = 0.0f;
	for(uint_8 i = 0; i < count; ++i){ //TODO: assuming count is = 4
		if(data[i] != nullptr) {
			variance += (*data[i]-mean)*(*data[i]-mean);
			num++;
		}
	}
	variance /= num;

	return sqrt(variance);
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
