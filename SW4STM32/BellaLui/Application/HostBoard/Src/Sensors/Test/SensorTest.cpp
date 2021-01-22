/*
 * SensorAcquisition.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include <gtest/gtest.h>

#include "MockIMU.h"
#include "MockBarometer.h"

#include "Sensors/UnbiasedIMU.h"
#include "Sensors/UnbiasedBarometer.h"


TEST(SensorTest, MainSensorTest) {
	MockIMU imu1("IMU 1");
	MockIMU imu2("IMU 2");
	MockIMU imu3("IMU 3");
	MockIMU imu4("IMU 4");

	MockBarometer barometer1("Barometer 1");
	MockBarometer barometer2("Barometer 2");
	MockBarometer barometer3("Barometer 3");
	MockBarometer barometer4("Barometer 4");

	UnbiasedIMU imu("Unbiased IMU", {&imu1, &imu2, &imu3, &imu4});
	UnbiasedBarometer barometer("Unbiased Barometer", {&barometer1, &barometer2, &barometer3, &barometer4});


	imu.load();
	barometer.load();

	IMUData imuData;
	BarometerData barometerData;

	if(imu.fetch(&imuData)) {

	} else {
		imu.reset();
	}


	/*if(!barometer.fetch(&barometerData)) {

	} else {
		barometer.reset();
	}*/

	ASSERT_EQ(1, 1);
}
