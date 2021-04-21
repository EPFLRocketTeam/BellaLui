/*
 * DataStructures.h
 *
 *  Created on: 22 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_DATASTRUCTURES_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_DATASTRUCTURES_H_


struct Vector {
	float x;
	float y;
	float z;
};

struct IMUData {
	Vector accel;
	Vector gyro;
};


struct BarometerData {
	float pressure;	   // hPa
	float temperature; // Centi-degrees
};

struct ThrustData {
	float thrust;
};


struct AltitudeData {
	float pressure;
	float temperature;
	float altitude;
	float base_pressure;
	float base_temperature;
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_DATASTRUCTURES_H_ */
