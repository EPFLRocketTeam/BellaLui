/*Using "barometric formula with temperature"
Ground altitude is 0m */

#include <cmath>
#include <cstdbool>
#include <Sensors/AltitudeEstimator.h>
#include <Sensors/sensor_calibration.h>

//#define DEBUG



float T0 = -1; //temperature at ground level [K]
float P0 = -1; //pressure at ground level [hPa]
float T0_averaged = 0;
float P0_averaged = 0;
bool constants_set = false; //true when constants have been set
int constants_iterator = 0;

/*
 * recalibrate_altitude_estimator
 * Resets the above global variables to restart a calibration process
 */

void recalibrate_altitude_estimator() {
	T0_averaged = 0;
	P0_averaged = 0;
	constants_set = false;
	constants_iterator = 0;
}

/*setConstants
Sets values for temperature and pressure at ground level.
To be executed only once
Inputs: temperature and pressure, addresses of T0 and P0
Outputs: -*/

static void setConstants(float temperature, float pressure, float* T0, float* P0) {
	if (constants_iterator >= N_ITERATIONS_AVERAGE)
		{
			*T0 = T0_averaged;
			*P0 = P0_averaged;
			constants_set = true;
		}

	//first time we set the first ever read value
	if (constants_iterator == 0){
		*T0 = temperature;
		*P0 = pressure;
	}

	//we set the ground values to be the average of the first N_ITERATIONS_AVERAGE values
	if (constants_iterator < N_ITERATIONS_AVERAGE){
		T0_averaged += temperature/N_ITERATIONS_AVERAGE;
		P0_averaged += pressure/N_ITERATIONS_AVERAGE;
		constants_iterator++;
	}
}

/*getPresFromALt
Returns pressure as calculated exclusively from altitude
Inputs: altitude [m]
Outputs: pressure [hPa] */
static float getPresFromAlt(float altitude) {
	return P0*pow((1 + A*altitude/T0), (G0/R/-A));
}

/*getAltFromPres
Returns altitude as calculated exclusively from pressure
Inputs: pressure [hPa]
Outputs: altitude [m]*/
static float getAltFromPres(float pressure) {
	return T0/A*(pow(pressure/P0, -A*R/G0) - 1);
}

/*getTempFromAlt
Returns temperature as calculated exclusively by altitude
Inputs: altitude [m]
Outputs: temperature [m] */
static float getTempFromAlt(float altitude) {
	return T0 + A*altitude;
}

/* altitudeComputation
Function that will loop indefinetly.
Returns altitude.
Inputs: pressure [hPa] and temperature [K] from sensors
Outputs: altitude [m] */



AltitudeEstimator::AltitudeEstimator(const char* identifier, Sensor<BarometerData>* sensor) : Sensor(identifier), barometer(sensor) {

}

bool AltitudeEstimator::load() {
	ready = true;
	return true;
}

bool AltitudeEstimator::unload() {
	ready = false;
	return true;
}

bool AltitudeEstimator::fetch(AltitudeData* data) {
	if(!ready)
		return false;

	BarometerData barodata;

	if(barometer->fetch(&barodata)) {
		data->pressure = barodata.pressure;
		data->temperature = barodata.temperature;
		data->altitude = altitudeComputation(barodata.pressure, barodata.temperature * 0.01f);

		if(constants_set) {
			data->base_pressure = P0;
			data->base_temperature = T0 * 100.0f; // °C to centi-°C
		}

		return true;
	} else {
		return false;
	}
}

float AltitudeEstimator::altitudeComputation(float raw_pressure, float raw_temperature) { // Temperature in °C, pressure can have any unit.
	float altitude1;
	float altitude2;
	float temperature1;
	float temp_increase;
	float pressure2;

	raw_temperature += 273.0f;

	//set constants
	if (constants_set == false) {
		setConstants(raw_temperature, raw_pressure, &T0, &P0);
	}

	//altitude (first approximation)
	altitude1 = getAltFromPres(raw_pressure);

	//temperature (first approximation)
	temperature1 = getTempFromAlt(altitude1);

	//increase in temperature
	temp_increase = (raw_temperature - temperature1)/T0; //barometric formula with temperature
	//temp_increase = (raw_temperature - temperature1)/raw_temperature; //barometric formula with temperature 2
	//temp_increase = (raw_temperature - temperature1)/temperature1; //barometric formula with temperature 3

	//decrease in pressure
	pressure2 = raw_pressure*(1 + temp_increase*TEMPERATURE_CORRECTION_FACTOR);

	altitude2 = getAltFromPres(pressure2);

	return altitude1; // return first approximation of altitude for now
}
