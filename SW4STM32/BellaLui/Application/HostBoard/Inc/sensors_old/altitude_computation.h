#define A -0.0065 // thermal lapse rate
#define R 287.0 //gas constant for air
#define G0 9.80665 //acceleration due to gravity

float T0 = -1; //temperature at ground level [K]
float P0 = -1; //pressure at ground level [hPa]
bool constants_set = false; //true when constants have been set

/* altitudeComputation
Function that will loop indefinetly.
Returns altitude.
Inputs: pressure [hPa] and temperature [K] from sensors
Outputs: altitude [m] */
float altitudeComputation(float raw_temperature, float raw_pressure);