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
	MockIMU imu1("NoisyIMU1.csv");
	MockIMU imu2("NoisyIMU2.csv");
	MockIMU imu3("NoisyIMU3.csv");
	MockIMU imu4("NoisyIMU4.csv");

	EXPECT_EQ(imu1.load(), true);
	EXPECT_EQ(imu2.load(), true);
	EXPECT_EQ(imu3.load(), true);
	EXPECT_EQ(imu4.load(), true);
}

TEST(SensorTest, BarometerFlightDataTest) {
	MockBarometer barometer1("NoisyBarometer1.csv");
	MockBarometer barometer2("NoisyBarometer2.csv");
	MockBarometer barometer3("NoisyBarometer3.csv");
	MockBarometer barometer4("NoisyBarometer4.csv");

	EXPECT_EQ(barometer1.load(), true);
	EXPECT_EQ(barometer2.load(), true);
	EXPECT_EQ(barometer3.load(), true);
	EXPECT_EQ(barometer4.load(), true);
}

TEST(SensorTest, BarometerRedundancyTest) {
	MockBarometer barometer1("NoisyBarometer1.csv");
	MockBarometer barometer2("NoisyBarometer2.csv");
	MockBarometer barometer3("NoisyBarometer3.csv");
	MockBarometer barometer4("NoisyBarometer4.csv");
	MockBarometer realBarometer("RealBarometer.csv");

	BarometerData noisedData;
	BarometerData realData;

	float accumPress;
	float accumTemp;
	uint32_t i;

	ASSERT_EQ(realBarometer.load(), true);
	ASSERT_EQ(barometer1.load(), true);
	ASSERT_EQ(barometer2.load(), true);
	ASSERT_EQ(barometer3.load(), true);
	ASSERT_EQ(barometer4.load(), true);

	accumPress = 0.0f;
	accumTemp = 0.0f;
	i = 0;
	while(barometer1.fetch(&noisedData) && realBarometer.fetch(&realData)) {
		float diffPress = noisedData.pressure - realData.pressure;
		float diffTemp = noisedData.temperature - realData.temperature;
		accumPress += diffPress * diffPress;
		accumTemp += diffTemp * diffTemp;
		i++;
	}

	float varPress1 = accumPress/i;
	float varTemp1 = accumTemp/i;

	ASSERT_EQ(realBarometer.reset(), true);

	accumPress = 0.0f;
	accumTemp = 0.0f;
	i = 0;
	while(barometer2.fetch(&noisedData) && realBarometer.fetch(&realData)) {
		float diffPress = noisedData.pressure - realData.pressure;
		float diffTemp = noisedData.temperature - realData.temperature;
		accumPress += diffPress * diffPress;
		accumTemp += diffTemp * diffTemp;
		i++;
	}

	float varPress2 = accumPress/i;
	float varTemp2 = accumTemp/i;

	ASSERT_EQ(realBarometer.reset(), true);

	accumPress = 0.0f;
	accumTemp = 0.0f;
	i = 0;
	while(barometer3.fetch(&noisedData) && realBarometer.fetch(&realData)) {
		float diffPress = noisedData.pressure - realData.pressure;
		float diffTemp = noisedData.temperature - realData.temperature;
		accumPress += diffPress * diffPress;
		accumTemp += diffTemp * diffTemp;
		i++;
	}

	float varPress3 = accumPress/i;
	float varTemp3 = accumTemp/i;

	ASSERT_EQ(realBarometer.reset(), true);

	accumPress = 0.0f;
	accumTemp = 0.0f;
	i = 0;
	while(barometer4.fetch(&noisedData) && realBarometer.fetch(&realData)) {
		float diffPress = noisedData.pressure - realData.pressure;
		float diffTemp = noisedData.temperature - realData.temperature;
		accumPress += diffPress * diffPress;
		accumTemp += diffTemp * diffTemp;
		i++;
	}

	float varPress4 = accumPress/i;
	float varTemp4 = accumTemp/i;

	ASSERT_EQ(realBarometer.reset(), true);
	ASSERT_EQ(barometer1.unload(), true);
	ASSERT_EQ(barometer2.unload(), true);
	ASSERT_EQ(barometer3.unload(), true);
	ASSERT_EQ(barometer4.unload(), true);

	UnbiasedBarometer unbiasedBarometer("Unbiased barometer", {&barometer1, &barometer2, &barometer3, &barometer4});

	ASSERT_EQ(unbiasedBarometer.load(), true);

	accumPress = 0.0f;
	accumTemp = 0.0f;
	i = 0;

	uint32_t numOutliers = 0;
	while(unbiasedBarometer.fetch(&noisedData) && realBarometer.fetch(&realData)) {
		float diffPress = noisedData.pressure - realData.pressure;
		float diffTemp = noisedData.temperature - realData.temperature;
		accumPress += diffPress * diffPress;
		accumTemp += diffTemp * diffTemp;

		if(abs(diffPress / realData.pressure) > 0.01f || abs(diffTemp / realData.temperature) > 0.05f) {
			numOutliers++;
		}

		i++;
	}

	EXPECT_LT(numOutliers, 13);
	/*
	 *  The probability of two outliers occurring at the same time is p*(1-(1-p)^3) where p is 0.01 in the generated dataset.
	 *  For the whole dataset, we multiply this probability by twice the size of the dataset. (10451*2)
	 *  We also take a safety margin of 2.
	 */

	float sigmaNoisedPress = sqrt(varPress1 + varPress2 + varPress3 + varPress4) / 4.0f;
	float sigmaNoisedTemp = sqrt(varTemp1 + varTemp2 + varTemp3 + varTemp4) / 4.0f;
	float sigmaUnbiasedPress = sqrt(accumPress/i);
	float sigmaUnbiasedTemp = sqrt(accumTemp/i);

	EXPECT_LT(sigmaUnbiasedPress, sigmaNoisedPress); // First check if there was an enhancement
	EXPECT_LT(sigmaUnbiasedTemp, sigmaNoisedTemp);

	EXPECT_LT(sigmaUnbiasedPress, sigmaNoisedPress * 0.3f); // Check if outliers were eliminated
	EXPECT_LT(sigmaUnbiasedTemp, sigmaNoisedTemp * 0.3f);
}

/*TEST(SensorTest, AltitudeEstimatorFullTest) {
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
}*/

TEST(SensorTest, RemoteSensorTest) {
	RemoteSensor<ThrustData> sensor("Thrust Remote Sensor");
	sensor.load();

	ThrustData data;
	data.thrust = 42.0f;

	EXPECT_EQ(sensor.fetch(&data), false);

	sensor.onDataReception(data);

	data.thrust = 0.0f;
	ASSERT_FLOAT_EQ(data.thrust, 0.0f);

	EXPECT_EQ(sensor.fetch(&data), true);

	ASSERT_FLOAT_EQ(data.thrust, 42.0f);

}
