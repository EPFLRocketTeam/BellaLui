/*
 * StandardI2CDriver.cpp
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */


#include <Sensors/I2CDriver.h>

#define I2C_TIMEOUT 3


int8_t StandardI2CDriver::read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t rslt = 0;

	vTaskSuspendAll();

	rslt = HAL_I2C_Mem_Read(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);

	xTaskResumeAll();

	return rslt;
}

int8_t StandardI2CDriver::write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t rslt = 0;

	vTaskSuspendAll();

	rslt = HAL_I2C_Mem_Write(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);

	xTaskResumeAll();

	return rslt;
}

void StandardI2CDriver::reset() {
	HAL_I2C_DeInit(&hi2c3);
	osDelay(10);
	hi2c3.Instance->CR1 |= I2C_CR1_SWRST | I2C_CR1_STOP;
	osDelay(10);
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
	osDelay(10);
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
	osDelay(10);
	hi2c3.Instance->CR1 &= ~(I2C_CR1_SWRST | I2C_CR1_STOP);
	osDelay(10);
	HAL_I2C_Init(&hi2c3);
	osDelay(10);
}
