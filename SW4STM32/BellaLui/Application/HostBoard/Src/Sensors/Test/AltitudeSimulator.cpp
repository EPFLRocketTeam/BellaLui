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

AltitudeSimulator::AltitudeSimulator(const char* identifier, uint32_t start, uint32_t end) : Sensor(identifier), start(start), end(end) {

}


bool AltitudeSimulator::load() {
	psimulation = fopen(INPUT_FILE_NAME, "r");
	return true;
}

bool AltitudeSimulator::unload() {
	fclose(psimulation);
	return true;
}

bool AltitudeSimulator::fetch(AltitudeData* data) {
	float altitude_read;
	float time_read;

	//open files

	//altitude calculation
	//read data from simulation
	do {
		fscanf(psimulation, "%f,%*f,%*f,%f,%*f,%*f,%*f,%*f,%*f", &time_read, &altitude_read);
	} while((uint32_t) time_read < start);


	if((uint32_t) time_read >= end) {
		return false;
	}

	data->altitude = altitude_read;

	return true;
}
