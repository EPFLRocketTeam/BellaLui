/*
 * UnbiasedSensor.h
 *
 *  Created on: 11 Nov 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_

#define THE_FURTHER_INDEX(data_ptr, sort_arr_ptr, index_small, index_large, central_value) ( abs(*data_ptr[sort_arr_ptr[index_large]]-central_value) > abs(*data_ptr[sort_arr_ptr[index_small]]-central_value) ? index_large : index_small )
#define IS_IN_INTERVAL(test_value, value_min, value_max) (test_value >= value_min && test_value <= value_max)

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
	void sorting_network(float** data, uint8_t* sorting_array);
	float standard_deviation(float** data);

};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDSENSOR_H_ */
