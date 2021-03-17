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
	float pressure;
	float temperature;
};

struct ThrustData {
	float thrust;
};


struct AltitudeData {
	float altitude;
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_DATASTRUCTURES_H_ */
