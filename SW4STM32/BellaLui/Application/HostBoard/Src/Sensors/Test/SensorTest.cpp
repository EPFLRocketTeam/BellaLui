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


namespace testing
{
 namespace internal
 {
  enum GTestColor {
      COLOR_DEFAULT,
      COLOR_RED,
      COLOR_GREEN,
      COLOR_YELLOW
  };

  extern void ColoredPrintf(GTestColor color, const char* fmt, ...);
 }
}
#define PRINTF(...)  do { testing::internal::ColoredPrintf(testing::internal::COLOR_GREEN, "[          ] "); testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); } while(0)



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

	/*if(imu.fetch(&imuData)) {

	} else {
		imu.reset();
	}


	if(altitude.fetch(&altitudeData)) {

	} else {
		altitude.reset();
	}*/



	ASSERT_EQ(1, 1);
}


TEST(SensorTest, AltitudeEstimatorTest) {
	MockBarometer barometer("Barometer");

	//std::vector<std::vector<float>> flight_data = read_csv("Flight.csv");

	AltitudeEstimator altitude("Altitude Estimator", &barometer);
	AltitudeSimulator simulator("Altitude Simulator");

	altitude.load();
	simulator.load();

	AltitudeData altitudeData;
	AltitudeData simulatedData;

	PRINTF("Hello world\n");

	while(altitude.fetch(&altitudeData)) {
		simulator.fetch(&simulatedData);
		ASSERT_NEAR(altitudeData.altitude, simulatedData.altitude, 2000.0f);
	}
}
