/*
 * SensorAcquisition.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include "Sensors/UnbiasedIMU.h"
#include "Sensors/UnbiasedBarometer.h"
#include "Sensors/Barometer.h"
#include "Sensors/IMU.h"
#include "Sensors/I2CDriver.h"

#include "debug/profiler.h"
#include "debug/led.h"
#include "debug/console.h"
#include "debug/monitor.h"
#include "can_transmission.h"
#include "sync.h"

#include <stdint.h>

void TK_sensor_acquisition(const void *argument) {
	uint16_t retry_counter = 1;

	uint8_t led_barometer = led_register_TK();
	uint8_t led_accelerometer = led_register_TK();

	StandardI2CDriver i2c;
	FastI2CDriver fmpi2c;

	IMU imu1("IMU 1", &i2c, BNO055_I2C_ADDR1);
	IMU imu2("IMU 2", &i2c, BNO055_I2C_ADDR2);
	IMU imu3("IMU 3", &fmpi2c, BNO055_I2C_ADDR1);
	IMU imu4("IMU 4", &fmpi2c, BNO055_I2C_ADDR2);

	Barometer barometer1("Barometer 1", &i2c, BME280_I2C_ADDR_PRIM);
	Barometer barometer2("Barometer 2", &i2c, BME280_I2C_ADDR_SEC);
	Barometer barometer3("Barometer 3", &fmpi2c, BME280_I2C_ADDR_PRIM);
	Barometer barometer4("Barometer 4", &fmpi2c, BME280_I2C_ADDR_SEC);

	UnbiasedIMU imu("Unbiased IMU", {&imu1, &imu2, &imu3, &imu4});
	UnbiasedBarometer barometer("Unbiased Barometer", {&barometer1, &barometer2, &barometer3, &barometer4});

	imu.load();
	barometer.load();

	IMUData imuData;
	BarometerData barometerData;

	while(true) {
		start_profiler(1); // Whole thread profiling


		start_profiler(1);
		if(imu.fetch(&imuData)) {
			led_set_TK_rgb(led_accelerometer, 0x00, 0xFF, 0x00);
		} else {
			imu.reset();
			rocket_log("%s ceased functioning\n", imu.name());
			led_set_TK_rgb(led_accelerometer, 0xFF, 0x00, 0x00);
		}
		end_profiler();


		start_profiler(2);
		if(!barometer.fetch(&barometerData)) {
			led_set_TK_rgb(led_barometer, 0x00, 0xFF, 0x00);
		} else {
			barometer.reset();
			rocket_log("%s ceased functioning\n", barometer.name());
			led_set_TK_rgb(led_barometer, 0xFF, 0x00, 0x00);
		}
		end_profiler();


		start_profiler(3);

		uint32_t time = HAL_GetTick();

		can_setFrame((int32_t) barometerData.temperature, DATA_ID_TEMPERATURE, time);
		can_setFrame((int32_t) (barometerData.pressure * 100), DATA_ID_PRESSURE, time);
		can_setFrame((int32_t) imuData.accel.x, DATA_ID_ACCELERATION_X, time);
		can_setFrame((int32_t) imuData.accel.y, DATA_ID_ACCELERATION_Y, time);
		can_setFrame((int32_t) imuData.accel.z, DATA_ID_ACCELERATION_Z, time);
		can_setFrame((int32_t) (1000 * imuData.gyro.x), DATA_ID_GYRO_X, time);
		can_setFrame((int32_t) (1000 * imuData.gyro.y), DATA_ID_GYRO_Y, time);
		can_setFrame((int32_t) (1000 * imuData.gyro.z), DATA_ID_GYRO_Z, time);

		end_profiler();

		end_profiler();


		if(enter_monitor(SENSOR_MONITOR)) {
			rocket_log(" Excluded barometer outputs: %d\x1b[K\n", barometer.getExcludedCount());
			rocket_log(" Excluded accelerometer outputs: %d\x1b[K\n", imu.getExcludedCount());
			rocket_log(" Temperature: %d [m°C]\x1b[K\n", (uint32_t) (barometerData.temperature * 10));
			rocket_log(" Pressure: %d [Pa]\x1b[K\n", (uint32_t) (barometerData.pressure));
			rocket_log(" Acceleration X: %d [mg]\x1b[K\n", (uint32_t) (imuData.accel.x));
			rocket_log(" Acceleration Y: %d [mg]\x1b[K\n", (uint32_t) (imuData.accel.y));
			rocket_log(" Acceleration Z: %d [mg]\x1b[K\n", (uint32_t) (imuData.accel.z));
			rocket_log(" Rotation X: %d [mrad/s]\x1b[K\n", (uint32_t) (1000 * imuData.gyro.x));
			rocket_log(" Rotation Y: %d [mrad/s]\x1b[K\n", (uint32_t) (1000 * imuData.gyro.y));
			rocket_log(" Rotation Z: %d [mrad/s]\x1b[K\n\x1b[K\n", (uint32_t) (1000 * imuData.gyro.z));

			exit_monitor(SENSOR_MONITOR);
		}

		sync_logic(10);
	}
}
