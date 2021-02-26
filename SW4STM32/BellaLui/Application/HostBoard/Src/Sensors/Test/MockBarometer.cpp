/*
 * Barometer.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "MockBarometer.h"

#include <iostream>


#define ARRAY_LENGTH 2121
#define INPUT_FILE_NAME "Flight.csv"


FILE* pflight;

MockBarometer::MockBarometer(const char* identifier) : Sensor(identifier) {

}


bool MockBarometer::load() {
	pflight = fopen(INPUT_FILE_NAME, "r");
	return true;
}

bool MockBarometer::reset() {
	fclose(pflight);
	return load();
}

bool MockBarometer::fetch(BarometerData* data) {
	int i;
	float pressure_read;
	float temperature_read;
	float time_read;
	float altitude_calculated;

	//open files

	//altitude calculation
	//read data from simulation
	fscanf(pflight, "%f,%*f,%f,%f,%*f,%*f,%*f,%*f,%*f,%*f", &time_read, &pressure_read, &temperature_read);

	if(time_read == EOF) {
		return false;
	}

	temperature_read += 273;
	//pressure_read /= 100;

	data->temperature = temperature_read;
	data->pressure = pressure_read;

	return true;
}
