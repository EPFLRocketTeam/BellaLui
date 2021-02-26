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


FILE* psimulation;

AltitudeSimulator::AltitudeSimulator(const char* identifier) : Sensor(identifier) {

}


bool AltitudeSimulator::load() {
	psimulation = fopen(INPUT_FILE_NAME, "r");
	return true;
}

bool AltitudeSimulator::reset() {
	fclose(psimulation);
	return load();
}

bool AltitudeSimulator::fetch(AltitudeData* data) {
	float altitude_read;
	float time_read;

	//open files

	//altitude calculation
	//read data from simulation
	fscanf(psimulation, "%*f,%f", &time_read, &altitude_read);

	if(time_read == EOF) {
		return false;
	}


	data->altitude = altitude_read;

	return true;
}
