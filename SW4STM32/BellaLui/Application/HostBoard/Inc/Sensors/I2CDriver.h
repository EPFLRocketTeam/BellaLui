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

typedef int8_t (*IOFunc8)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len);
typedef int8_t (*IOFunc16)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

class I2CDriver {
public:
	I2CDriver(IOFunc8 read8, IOFunc16 read16, IOFunc8 write8, IOFunc16 write16)
		: readFunc8(read8), readFunc16(read16), writeFunc8(write8), writeFunc16(write16) {}

	IOFunc8 readFunc8;
	IOFunc16 readFunc16;
	IOFunc8 writeFunc8;
	IOFunc16 writeFunc16;

	static void wait(uint32_t ms) { osDelay(ms); }
	virtual void reset() {};
};

class StandardI2CDriver : public I2CDriver {
public:
	StandardI2CDriver();

	static int8_t read8(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len) {
		return read16(dev_id, reg_addr, data, len);
	}
	static int8_t read16(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

	static int8_t write8(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len) {
		return write16(dev_id, reg_addr, data, len);
	}
	static int8_t write16(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

	void reset();
};

class FastI2CDriver : public I2CDriver {
public:
	FastI2CDriver();

	static int8_t read8(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len) {
		return read16(dev_id, reg_addr, data, len);
	}
	static int8_t read16(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

	static int8_t write8(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len) {
		return write16(dev_id, reg_addr, data, len);
	}
	static int8_t write16(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

	void reset();
};

#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_I2CDRIVER_H_ */
