/*
 * Barometer.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "MockBarometer.h"

MockBarometer::MockBarometer(const char* identifier) : Sensor(identifier) {

}


bool MockBarometer::load() {
	return true;
}

bool MockBarometer::reset() {
	return load();
}

bool MockBarometer::fetch(BarometerData* data) {
	return true;
}
