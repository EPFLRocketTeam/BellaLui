/*
 * Barometer.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "MockBarometer.h"


#define ARRAY_LENGTH 2121
#define INPUT_FILE_NAME "Flight.csv"


MockBarometer::MockBarometer(const char* identifier) : MockBarometer(identifier, 0, 100) {

}

MockBarometer::MockBarometer(const char* identifier, uint32_t start, uint32_t end) : Sensor(identifier), file(nullptr), start(start), end(end) {

}


bool MockBarometer::load() {
	file = fopen(name(), "r");
	return file != nullptr;
}

bool MockBarometer::unload() {
	return file != nullptr && fclose(file) == 0;
}

bool MockBarometer::fetch(BarometerData* data) {
	float time_read;

	do {
		fscanf(file, "%f,%f,%f", &time_read, &data->pressure, &data->temperature);
	} while((uint32_t) time_read < start);

	if((uint32_t) time_read >= end) {
		return false;
	}

	return true;
}
