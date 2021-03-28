/*
 * IMU.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include <Sensors/IMU.h>
#include "debug/console.h"

#define ACCEL_ACQUISITION_DIVIDER 1
#define GYRO_ACQUISITION_DIVIDER 10



IMU::IMU(const char* identifier, I2CDriver* driver, uint8_t address) : Sensor(identifier), driver(driver) {
	this->dev.dev_addr = address;
	this->dev.bus_read = driver->readFunc;
	this->dev.bus_write = driver->writeFunc;
	this->dev.delay_msec = (void(*)(u32)) &driver->wait;
}


bool IMU::load() {
	int8_t result = bno055_init(&dev); // Returns 1 if error

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %s failed to initialise with error code %d\n", name(), result);
		return false;
	}

	result = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %s failed to set power mode with error code %d\n", name(), result);
		return false;
	}

	result = bno055_set_operation_mode(BNO055_OPERATION_MODE_ACCGYRO);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %s failed to set operation mode with error code %d\n", name(), result);
		return false;
	}

	result = bno055_set_accel_range(BNO055_ACCEL_RANGE_16G);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %s failed to set acceleration range with error code %d\n", name(), result);
		return false;
	}

	result = bno055_set_accel_bw(BNO055_ACCEL_BW_250HZ);
	// result = bno055_set_accel_bw(BNO055_ACCEL_BW_500HZ);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %s failed to set acceleration bandwidth with error code %d\n", name(), result);
		return false;
	}

	result = bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %s failed to set acceleration unit with error code %d\n", name(), result);
		return false;
	}

	result = bno055_set_gyro_bw(BNO055_GYRO_BW_230HZ);
	// result = bno055_set_gyro_bw(BNO055_GYRO_BW_523HZ);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %s failed to set gyroscope bandwidth with error code %d\n", name(), result);
		return false;
	}

	result = bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %s failed to set gyroscope unit with error code %d\n", name(), result);
		return false;
	}

	result = bno055_write_page_id(BNO055_PAGE_ZERO);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %s failed to set default write page ID with error code %d\n", name(), result);
		return false;
	}

	ready = true;

	return BNO055_SUCCESS;
}

bool IMU::unload() {
	this->driver->reset();

	ready = false;

	return true;
}

bool IMU::fetch(IMUData* data) {
	if(!ready) {
		return false;
	}

	static uint8_t accel_data[BNO055_ACCEL_XYZ_DATA_SIZE];
	static uint8_t gyro_data[BNO055_GYRO_XYZ_DATA_SIZE];
	static uint8_t accel_counter = 0;
	static uint8_t gyro_counter = 0;

	uint8_t result = 0;

	if(accel_counter++ == ACCEL_ACQUISITION_DIVIDER) {
		result += this->driver->readFunc(this->dev.dev_addr, BNO055_ACCEL_DATA_X_LSB_VALUEX_REG, accel_data, BNO055_ACCEL_XYZ_DATA_SIZE);
		accel_counter = 0;
	}

	if(gyro_counter++ == GYRO_ACQUISITION_DIVIDER) {
		result += this->driver->readFunc(this->dev.dev_addr, BNO055_GYRO_DATA_X_LSB_VALUEX_REG, gyro_data, BNO055_GYRO_XYZ_DATA_SIZE);
		gyro_counter = 0;
	}

	if(result != BNO055_SUCCESS) {
		ready = false;
		return false;
	}

	data->accel.x = (float) ((((int16_t) ((int8_t) accel_data[1]) << 8) | accel_data[0]) / BNO055_ACCEL_DIV_MG);
	data->accel.y = (float) ((((int16_t) ((int8_t) accel_data[3]) << 8) | accel_data[2]) / BNO055_ACCEL_DIV_MG);
	data->accel.z = (float) ((((int16_t) ((int8_t) accel_data[5]) << 8) | accel_data[4]) / BNO055_ACCEL_DIV_MG);
	data->gyro.x = (float) ((((int16_t) ((int8_t) gyro_data[1]) << 8) | gyro_data[0]) / BNO055_GYRO_DIV_RPS);
	data->gyro.y = (float) ((((int16_t) ((int8_t) gyro_data[3]) << 8) | gyro_data[2]) / BNO055_GYRO_DIV_RPS);
	data->gyro.z = (float) ((((int16_t) ((int8_t) gyro_data[5]) << 8) | gyro_data[4]) / BNO055_GYRO_DIV_RPS);

	return true;
}
