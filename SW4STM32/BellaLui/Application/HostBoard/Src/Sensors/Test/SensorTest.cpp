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

#include "Sensors/RemoteSensor.h"
#include "Sensors/UnbiasedIMU.h"
#include "Sensors/UnbiasedBarometer.h"
#include "Sensors/AltitudeEstimator.h"



TEST(SensorTest, IMUFlightDataTest) {
	MockIMU imu1("IMU.csv");
	MockIMU imu2("IMU.csv");
	MockIMU imu3("IMU.csv");
	MockIMU imu4("IMU.csv");

	EXPECT_EQ(imu1.load(), true);
	EXPECT_EQ(imu2.load(), true);
	EXPECT_EQ(imu3.load(), true);
	EXPECT_EQ(imu4.load(), true);
}

TEST(SensorTest, BarometerFlightDataTest) {
	MockBarometer barometer1("FakeBarometer1.csv");
	MockBarometer barometer2("FakeBarometer2.csv");
	MockBarometer barometer3("FakeBarometer3.csv");
	MockBarometer barometer4("FakeBarometer4.csv");

	EXPECT_EQ(barometer1.load(), true);
	EXPECT_EQ(barometer2.load(), true);
	EXPECT_EQ(barometer3.load(), true);
	EXPECT_EQ(barometer4.load(), true);
}

TEST(SensorTest, BarometerRedundancyTest) {
	MockBarometer barometer1("FakeBarometer1.csv");
	MockBarometer barometer2("FakeBarometer2.csv");
	MockBarometer barometer3("FakeBarometer3.csv");
	MockBarometer barometer4("FakeBarometer4.csv");

	UnbiasedBarometer barometer("Unbiased barometer", {&barometer1, &barometer2, &barometer3, &barometer4});


	ASSERT_EQ(barometer.load(), true);
}

TEST(SensorTest, AltitudeEstimatorFullTest) {
	MockBarometer barometer("RealBarometer.csv", 0, 104);
	AltitudeSimulator simulator("SimulatedFlight.csv", 0, 104);

	AltitudeEstimator altitude("Altitude Estimator", &barometer);

	ASSERT_EQ(barometer.load(), true);
	ASSERT_EQ(altitude.load(), true);
	ASSERT_EQ(simulator.load(), true);

	AltitudeData altitudeData;
	AltitudeData simulatedData;

	while(altitude.fetch(&altitudeData)) {
		simulator.fetch(&simulatedData);
		ASSERT_NEAR(altitudeData.altitude, simulatedData.altitude, 100.0f);
	}
}

TEST(SensorTest, AltitudeEstimatorAscentTest) {
	MockBarometer barometer("RealBarometer.csv", 0, 40);
	AltitudeSimulator simulator("SimulatedFlight.csv", 0, 40);

	AltitudeEstimator altitude("Altitude Estimator", &barometer);

	ASSERT_EQ(barometer.load(), true);
	ASSERT_EQ(altitude.load(), true);
	ASSERT_EQ(simulator.load(), true);

	AltitudeData altitudeData;
	AltitudeData simulatedData;

	while(altitude.fetch(&altitudeData)) {
		simulator.fetch(&simulatedData);
		ASSERT_NEAR(altitudeData.altitude, simulatedData.altitude, 30.0f);
	}
}

TEST(SensorTest, AltitudeEstimatorApogeeTest) {
	MockBarometer barometer("RealBarometer.csv", 37, 40);
	AltitudeSimulator simulator("SimulatedFlight.csv", 37, 40);

	AltitudeEstimator altitude("Altitude Estimator", &barometer);

	ASSERT_EQ(barometer.load(), true);
	ASSERT_EQ(altitude.load(), true);
	ASSERT_EQ(simulator.load(), true);

	AltitudeData altitudeData;
	AltitudeData simulatedData;

	while(altitude.fetch(&altitudeData)) {
		simulator.fetch(&simulatedData);
		ASSERT_NEAR(altitudeData.altitude, simulatedData.altitude, 10.0f);
	}
}

TEST(SensorTest, RemoteSensorTest) {
	RemoteSensor<ThrustData> sensor("Thrust Remote Sensor");

	ThrustData data;
	data.thrust = 42.0f;

	EXPECT_EQ(sensor.fetch(&data), false);

	sensor.onDataReception(data);

	data.thrust = 0.0f;
	ASSERT_FLOAT_EQ(data.thrust, 0.0f);

	EXPECT_EQ(sensor.fetch(&data), true);

	ASSERT_FLOAT_EQ(data.thrust, 42.0f);

}
