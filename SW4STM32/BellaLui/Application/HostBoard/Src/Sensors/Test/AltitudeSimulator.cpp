/*
 * AltitudeSimulator.cpp
 *
 *  Created on: 22 Jan 2021
 *      Author: Arion
 */


#include <iostream>
#include <cmath>

#include "AltitudeSimulator.h"


#define INPUT_FILE_NAME "SimulatedFlight.csv"


AltitudeSimulator::AltitudeSimulator(const char* identifier, uint32_t start, uint32_t end) : Sensor(identifier), file(nullptr), start(start), end(end) {

}


bool AltitudeSimulator::load() {
	file = fopen(name(), "r");
	return file != nullptr;
}

bool AltitudeSimulator::unload() {
	return file != nullptr && fclose(file) == 0;
}

bool AltitudeSimulator::fetch(AltitudeData* data) {
	float time_read;

	do {
		fscanf(file, "%f,%*f,%*f,%f,%*f,%*f,%*f,%*f,%*f", &time_read, &data->altitude);
	} while((uint32_t) time_read < start);

	if((uint32_t) time_read >= end) {
		return false;
	}

	return true;
}
