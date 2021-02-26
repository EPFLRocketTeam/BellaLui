#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_ALTITUDEESTIMATOR_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_ALTITUDEESTIMATOR_H_

#include "Sensors/UnbiasedSensor.h"


#define A -0.0065 // thermal lapse rate
#define R 287.0 //gas constant for air
#define G0 9.80665 //acceleration due to gravity
#define N_ITERATIONS_AVERAGE 50 //Number of iterations to set the ground values
#define TEMPERATURE_CORRECTION_FACTOR 0.5


/* altitudeComputation
Function that will loop indefinetly.
Returns altitude.
Inputs: pressure [hPa] and temperature [K] from sensors
Outputs: altitude [m] */


class AltitudeEstimator : public Sensor<AltitudeData> {
public:
	AltitudeEstimator(const char* identifier, Sensor<BarometerData>* sensor);

	bool load();
	bool reset();
	bool fetch(AltitudeData* data);

protected:
	float altitudeComputation(float raw_temperature, float raw_pressure);

private:
	Sensor<BarometerData>* barometer;
};






#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_ALTITUDEESTIMATOR_H_ */
