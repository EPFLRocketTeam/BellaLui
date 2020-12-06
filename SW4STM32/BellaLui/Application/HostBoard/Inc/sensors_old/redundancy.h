/*
 * redundancy.h
 *
 *  Created on: 25 Aug 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_REDUNDANCY_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_REDUNDANCY_H_

#define NORMAL_TRESHOLD 3.000   // 99 % confidence interval


#include <stdint.h>
#include <stdbool.h>


float get_filtered_sensor_output(float* values, uint8_t num_sensors);


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_REDUNDANCY_H_ */
