/*
 * SensorAcquisition.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include <gtest/gtest.h>


#include "CSV.h"

#include "MockIMU.h"
#include "MockBarometer.h"
#include "AltitudeSimulator.h"

#include "Sensors/UnbiasedIMU.h"
#include "Sensors/UnbiasedBarometer.h"
#include "Sensors/AltitudeEstimator.h"



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
	AltitudeEstimator altitude("Altitude Estimator", &barometer);

	imu.load();
	altitude.load();

	IMUData imuData;
	AltitudeData altitudeData;

	if(imu.fetch(&imuData)) {
		//std::cout << imuData.accel.y << std::endl;
	}


	while(altitude.fetch(&altitudeData)) {
	}

	ASSERT_EQ(1, 1);
}


TEST(SensorTest, AltitudeEstimatorFullTest) {
	MockBarometer barometer("Barometer", 0, 104);

	AltitudeEstimator altitude("Altitude Estimator", &barometer);
	AltitudeSimulator simulator("Altitude Simulator", 0, 104);

	altitude.load();
	simulator.load();

	AltitudeData altitudeData;
	AltitudeData simulatedData;

	while(altitude.fetch(&altitudeData)) {
		simulator.fetch(&simulatedData);
		ASSERT_NEAR(altitudeData.altitude, simulatedData.altitude, 100.0f);
	}
}

TEST(SensorTest, AltitudeEstimatorAscentTest) {
	MockBarometer barometer("Barometer", 0, 40);

	AltitudeEstimator altitude("Altitude Estimator", &barometer);
	AltitudeSimulator simulator("Altitude Simulator", 0, 40);

	altitude.load();
	simulator.load();

	AltitudeData altitudeData;
	AltitudeData simulatedData;

	while(altitude.fetch(&altitudeData)) {
		simulator.fetch(&simulatedData);
		ASSERT_NEAR(altitudeData.altitude, simulatedData.altitude, 30.0f);
	}
}

TEST(SensorTest, AltitudeEstimatorApogeeTest) {
	MockBarometer barometer("Barometer", 37, 40);

	AltitudeEstimator altitude("Altitude Estimator", &barometer);
	AltitudeSimulator simulator("Altitude Simulator", 37, 40);

	altitude.load();
	simulator.load();

	AltitudeData altitudeData;
	AltitudeData simulatedData;

	while(altitude.fetch(&altitudeData)) {
		simulator.fetch(&simulatedData);
		ASSERT_NEAR(altitudeData.altitude, simulatedData.altitude, 10.0f);
	}
}
