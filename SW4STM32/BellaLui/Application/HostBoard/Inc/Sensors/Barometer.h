/*
 * Barometer.h
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_BAROMETER_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_BAROMETER_H_


#include <Sensors/BME280/bme280.h>
#include <Sensors/I2CDriver.h>
#include <Sensors/Sensor.h>


struct BarometerData {
	float pressure;
	float temperature;
};

class Barometer : public Sensor<BarometerData> {
public:
	Barometer(const char* identifier, I2CDriver* driver, uint8_t address);

	bool load();
	bool reset();
	bool fetch(BarometerData* data);

private:
	I2CDriver* driver;
	bme280_dev dev;
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_BAROMETER_H_ */
