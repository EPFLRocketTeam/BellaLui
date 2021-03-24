/*
 * StandardI2CDriver.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */


#include <Sensors/I2CDriver.h>

#define FMPI2C_TIMEOUT 3

FastI2CDriver::FastI2CDriver() : I2CDriver(&FastI2CDriver::read, &FastI2CDriver::write) {}

int8_t FastI2CDriver::read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t rslt = 0;

	vTaskSuspendAll();

	rslt = HAL_FMPI2C_Mem_Read(&hfmpi2c1, dev_id << 1, reg_addr, FMPI2C_MEMADD_SIZE_8BIT, data, len, FMPI2C_TIMEOUT);

	xTaskResumeAll();

	return rslt;
}

int8_t FastI2CDriver::write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t rslt = 0;

	vTaskSuspendAll();

	rslt = HAL_FMPI2C_Mem_Write(&hfmpi2c1, dev_id << 1, reg_addr, FMPI2C_MEMADD_SIZE_8BIT, data, len, FMPI2C_TIMEOUT);

	xTaskResumeAll();

	return rslt;
}

void FastI2CDriver::reset() {
	HAL_FMPI2C_DeInit(&hfmpi2c1);
	osDelay(10);
	hfmpi2c1.Instance->CR1 |= I2C_CR1_SWRST | I2C_CR1_STOP;
	osDelay(10);
	RCC->APB1RSTR |= RCC_APB1RSTR_FMPI2C1RST;
	osDelay(10);
	RCC->APB1RSTR &= ~RCC_APB1RSTR_FMPI2C1RST;
	osDelay(10);
	hfmpi2c1.Instance->CR1 &= ~(I2C_CR1_SWRST | I2C_CR1_STOP);
	osDelay(10);
	HAL_FMPI2C_Init(&hfmpi2c1);
	osDelay(10);
}
