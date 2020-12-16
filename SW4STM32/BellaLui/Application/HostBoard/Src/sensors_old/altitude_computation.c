/*Using "barometric formula with temperature"
Ground altitude is 0m */

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include "altitude_computation.h"

//#define DEBUG

/*setConstants
Sets values for temperature and pressure at ground level.
To be executed only once
Inputs: temperature and pressure, addresses of T0 and P0
Outputs: -*/

void setConstants(float temperature, float pressure, float* T0, float* P0)
{
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
float getPresFromAlt(float altitude)
{
	return P0*pow((1 + A*altitude/T0), (G0/R/-A));
}

/*getAltFromPres
Returns altitude as calculated exclusively from pressure
Inputs: pressure [hPa]
Outputs: altitude [m]*/
float getAltFromPres(float pressure)
{
	return T0/A*(pow(pressure/P0, -A*R/G0) - 1);
}

/*getTempFromAlt
Returns temperature as calculated exclusively by altitude
Inputs: altitude [m]
Outputs: temperature [m] */
float getTempFromAlt(float altitude)
{
	return T0 + A*altitude;
}

/* altitudeComputation
Function that will loop indefinetly.
Returns altitude.
Inputs: pressure [hPa] and temperature [K] from sensors
Outputs: altitude [m] */
float altitudeComputation(float raw_temperature, float raw_pressure)
{
	float altitude1;
	float temperature1;
	float temp_increase;
	float pressure2;

	//set constants
	if (constants_set == false){
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

	return getAltFromPres(pressure2);
}


int main(void)
{
	#ifdef DEBUG
	float altitude = 0;

	altitude= altitudeComputation(285, 900);
	altitude= altitudeComputation(284, 850);
	printf("%f\n", altitude);
	#endif

	return 0;
}
