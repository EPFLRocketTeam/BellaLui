#define A -0.0065 // thermal lapse rate
#define R 287.0 //gas constant for air
#define G0 9.80665 //acceleration due to gravity
#define N_ITERATIONS_AVERAGE 50 //Number of iterations to set the ground values
#define TEMPERATURE_CORRECTION_FACTOR 0.5

float T0 = -1; //temperature at ground level [K]
float P0 = -1; //pressure at ground level [hPa]
float T0_averaged = 0;
float P0_averaged = 0;
bool constants_set = false; //true when constants have been set
int constants_iterator = 0;

/* altitudeComputation
Function that will loop indefinetly.
Returns altitude.
Inputs: pressure [hPa] and temperature [K] from sensors
Outputs: altitude [m] */
float altitudeComputation(float raw_temperature, float raw_pressure);
