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
#include <cmsis_os.h>

typedef int8_t (*IOFunc)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len);

class I2CDriver {
public:
	I2CDriver(IOFunc read, IOFunc write): readFunc(read), writeFunc(write) {}

	IOFunc readFunc;
	IOFunc writeFunc;

	static void wait(uint32_t ms) { osDelay(ms); }
	virtual void reset() {};
};

class StandardI2CDriver : public I2CDriver {
public:
	StandardI2CDriver();

	static int8_t read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len);
	static int8_t write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len);
	void reset();
};

class FastI2CDriver : public I2CDriver {
public:
	FastI2CDriver();

	static int8_t read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len);
	static int8_t write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len);
	void reset();
};

#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_I2CDRIVER_H_ */
