/*
 * redundancy.c
 *
 *  Created on: 25 Feb 2020
 *      Author: Quentin Delfosse
 */

#include <math.h>
#include <sensors_old/redundancy.h>
#include <sensors_old/sensors.h>

/*
 * sensor_elimination :
 *
 * Used to compare the data from the different sensors
 *
 * Input Arguments :
 *  - values from the different sensors
 *  - correct value determined by the algorithm
 *  - array of size MAX_SENSOR_NUMBER to know if there was an initialization or
 *    fetch error
 *
 * If sensor_elimination_4 finds an erroneous sensor, its values are not taken into
 * account and sensor_elimination_3 takes over. Indeed, the intervals of confidence
 * calculated by sensor_elimination _4 might have been corrupted by the erroneous sensor,
 * therefore new intervals have to be calculated.
 *
 *
 * O(n * 2^n) where n is the number of sensors
 *
 */

bool within_conf_interval(uint8_t index, float* values, uint8_t mask, uint8_t num_sensors);

// Combinations for 4 sensors. Lookup table to improve performance.
uint8_t combinations[] = {
	0b1111,
	0b0111,
	0b1011,
	0b1101,
	0b1110,
	/*0b0011,
	0b0101,
	0b0110,
	0b1100,
	0b1010,
	0b1001,
	0b1000,	The last combinations would mean that 2/4 sensors have ceased functioning
	0b0100,
	0b0010,
	0b0001,
	0b0000,*/
};


float get_filtered_sensor_output(float* values, uint8_t num_sensors) {

	float validated_values[num_sensors];
	uint8_t num_validated_values;

	for(uint8_t i = 0; i < sizeof(combinations); i++) {
		uint8_t mask = combinations[i];

		for(uint8_t j = 0; j < num_sensors; j++) {
			if(within_conf_interval(i, values, mask, num_sensors)) {
				validated_values[num_validated_values++] = values[j]; // Validates as many sensors as possible
			}
		}

		if(num_validated_values >= num_sensors) {
			break; // Values are now validated!
		} else {
			num_validated_values = 0; // Otherwise, reset.
		}
	}

	// Compute the mean of the validated values

	float mean = 0.0f;

	for(uint8_t i = 0; i < num_validated_values; i++) {
		mean += validated_values[i];
	}

	return mean / num_validated_values;
}

/*
 * within_conf_interval
 *
 * Checks if the FIRST GIVEN ARGUMENT is included inside [-3*sigma;+3*sigma]
 * of the normal distribution for a 99% confidence interval
 * The normal distribution is based on the values of the other
 * sensors
 *
 */


bool within_conf_interval(uint8_t index, float* values, uint8_t mask, uint8_t num_sensors) {
	if(num_sensors == 1) {
		return true;
	}

	float sum = 0.0f;
	uint8_t count = 0;

	for(uint8_t i = 0; i < num_sensors; i++) {
		if(i != index && (mask >> i) % 2 == 0) {
			sum += values[i];
			count++;
		}
	}

	float mean = sum / count;
	float variance = 0.0f;

	for(uint8_t i = 0; i < num_sensors; i++) {
		if(i != index && (mask >> i) % 2 == 0) {
			variance += (values[i] - mean) * (values[i] - mean);
		}
	}

	float deviation = sqrt(variance / count);

	float normalised_data = (values[index] - mean) / deviation;

	return -NORMAL_TRESHOLD < normalised_data && normalised_data < +NORMAL_TRESHOLD;
}
