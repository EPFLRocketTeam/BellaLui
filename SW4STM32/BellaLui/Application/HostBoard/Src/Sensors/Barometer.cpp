/*
 * Barometer.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#include <Sensors/Barometer.h>

Barometer::Barometer(const char* identifier, I2CDriver* driver, uint8_t address) : Sensor(identifier), driver(driver) {
	this->dev.dev_id = address;
	this->dev.intf = BME280_I2C_INTF;
	this->dev.read = &driver->read;
	this->dev.write = &driver->write;
	this->dev.delay_ms = &driver->wait;
}


bool Barometer::load() {
	int8_t result = bme280_init(&dev); // Returns 1 if error

	if(result != BME280_OK) {
		// rocket_log("Barometer %u failed to initialise with error code %d\n", sensor_id, result);
		return result;
	}
	//Always read the current settings before writing
	result = bme280_get_sensor_settings(&dev);

	if(result != BME280_OK) {
		// rocket_log("Barometer %u failed to fetch sensor settings with error code %d\n", sensor_id, result);
		return result;
	}
	//Overwrite the desired settings
	dev.settings.filter = BME280_FILTER_COEFF_OFF;
	dev.settings.osr_p = BME280_OVERSAMPLING_8X;
	dev.settings.osr_t = BME280_OVERSAMPLING_1X;
	dev.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;


	result = bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &dev);

	if(result != BME280_OK) {
		// rocket_log("Barometer %u failed to change sensor settings with error code %d\n", sensor_id, result);
		return result;
	}

	//Always set the power mode after setting the configuration
	result = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

	if(result != BME280_OK) {
		// rocket_log("Barometer %u failed to change power mode with error code %d\n", sensor_id, result);
		return result;
	}

	return BME280_OK;
}

bool Barometer::reset() {
	this->driver->reset();
	return load();
}

bool Barometer::fetch(BarometerData* data) {
	static struct bme280_data raw_data;

	int8_t result = bme280_get_sensor_data(BME280_TEMP | BME280_PRESS, &raw_data, &dev);

	data->temperature = (float) raw_data.temperature;
	data->pressure = (float) raw_data.pressure / 100;

	return result;
}
