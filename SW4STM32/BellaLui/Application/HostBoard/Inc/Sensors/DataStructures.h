/*
 * DataStructures.h
 *
 *  Created on: 22 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_DATASTRUCTURES_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_DATASTRUCTURES_H_



struct AltitudeData {
	float altitude;
};


struct BarometerData {
	float pressure;
	float temperature;
};


struct Vector {
	float x;
	float y;
	float z;
};

struct IMUData {
	Vector accel;
	Vector gyro;
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_DATASTRUCTURES_H_ */
