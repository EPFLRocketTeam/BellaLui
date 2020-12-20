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
	uint8_t sorting_array[SENSORS_NB] = {0,1,2,3}; //TODO:magic numbers? could work with array of pointers of data instead of indexes... faster code I believe but more RAM (insignificant?)
	sortingNetwork(data, sorting_array);

	return filterOutData(data,sorting_array,0,3);
}

template<class T>
void UnbiasedSensor<T>::sortingNetwork(float** data, uint8_t* sorting_array) { //TODO: can use sorting network? fixed amount of sensors?
    if(*data[sorting_array[0]] > *data[sorting_array[2]])		swap(sorting_array[0],sorting_array[2]);
    if(*data[sorting_array[1]] > *data[sorting_array[3]]) 		swap(sorting_array[1],sorting_array[3]);
    if(*data[sorting_array[0]] > *data[sorting_array[1]])		swap(sorting_array[0],sorting_array[1]);
    if(*data[sorting_array[2]] > *data[sorting_array[3]]) 		swap(sorting_array[2],sorting_array[3]);
    if(*data[sorting_array[1]] > *data[sorting_array[2]])		swap(sorting_array[1],sorting_array[2]);
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
