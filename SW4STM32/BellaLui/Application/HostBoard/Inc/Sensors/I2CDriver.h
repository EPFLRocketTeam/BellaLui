/*
 * I2CDriver.h
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_I2CDRIVER_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_I2CDRIVER_H_

#include <stdint.h>
#include <i2c.h>
#include <fmpi2c.h>

class I2CDriver {
public:
	virtual ~I2CDriver() {}

	static int8_t read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
	static int8_t write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
	static void wait(uint32_t ms) { osDelay(ms); }
	static void reset();
};

class StandardI2CDriver : I2CDriver {
public:
	static int8_t read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
	static int8_t write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
	static void reset();
};

class FastI2CDriver : I2CDriver {
public:
	static int8_t read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
	static int8_t write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
	static void reset();
};

#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_I2CDRIVER_H_ */
