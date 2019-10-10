#include "Sensors/sensor_board.h"
#include "Sensors/BME280/bme280.h"
#include "Sensors/BNO055/bno055.h"
#include "CAN_communication.h"
#include "Misc/Common.h"
#include "Misc/rocket_constants.h"
#include "led.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>

#define I2C_TIMEOUT 3
#define BARO_CALIB_N 128

int8_t init_bme();
int8_t init_bno();
int8_t fetch_bme();
int8_t fetch_bno();

int8_t stm32_i2c_read (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_i2c_write (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void stm32_delay_ms (uint32_t delay);

extern I2C_HandleTypeDef hi2c3;

char buf[300];

//## BME280 ##
struct bme280_dev bme;
struct bme280_data bme_data;
uint32_t bme_basepressure;
uint32_t bme_calib_counter;

//## BNO055 ##
struct bno055_t bno055;
struct bno055_accel_float_t accel;
struct bno055_gyro_float_t gyro;
struct bno055_mag_float_t mag;

int led_sensor_id_imu, led_sensor_id_baro;

bool bme_calibrated = false;

int set_sensor_led(int id, int flag) {
	if (flag) { // success
		led_set_TK_rgb(id, 0, 50, 0);
	} else { // fail
		led_set_TK_rgb(id, 500, 0, 0);
	}
	return flag;
}

//Sensor Board thread
void TK_sensor_board(void const * argument)
{
	int imu_init = 0, baro_init=0;
	osDelay(500);

	led_sensor_id_imu  = led_register_TK();
	led_sensor_id_baro = led_register_TK();

	for(;;) {
		if (imu_init) {
			set_sensor_led(led_sensor_id_imu, fetch_bno() == BNO055_SUCCESS);
		} else {
			imu_init = set_sensor_led(led_sensor_id_imu, init_bno() == BNO055_SUCCESS);
		}

		if (baro_init) {
			set_sensor_led(led_sensor_id_baro, fetch_bme() == BME280_OK);
		} else {
			baro_init = set_sensor_led(led_sensor_id_baro, init_bme() == BME280_OK);
		}

		osDelay(10);

		if(!baro_init && !imu_init) {
			osDelay(1000);
		}
	}
}

int8_t init_bme()
{
	int8_t rslt;

	bme.dev_id = BME280_I2C_ADDR_PRIM;
	bme.intf = BME280_I2C_INTF;
	bme.read = &stm32_i2c_read;
	bme.write = &stm32_i2c_write;
	bme.delay_ms = &stm32_delay_ms;

	rslt = bme280_init(&bme);
	if(rslt != BME280_OK)
		return rslt;

	//Always read the current settings before writing
	rslt = bme280_get_sensor_settings(&bme);
	if(rslt != BME280_OK)
		return rslt;

	//Overwrite the desired settings
	bme.settings.filter = BME280_FILTER_COEFF_OFF;
	bme.settings.osr_p = BME280_OVERSAMPLING_8X;
	bme.settings.osr_t = BME280_OVERSAMPLING_1X;
	bme.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

	rslt = bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &bme);
	if(rslt != BME280_OK)
		return rslt;

	//Always set the power mode after setting the configuration
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme);

	bme_calibrated = false;
	bme_calib_counter = 0;
	bme_basepressure = 0;
	return rslt;
}

int8_t init_bno()
{
	int8_t rslt;
	bno055.bus_write = &stm32_i2c_write;
	bno055.bus_read = &stm32_i2c_read;
	bno055.delay_msec = &stm32_delay_ms;
	bno055.dev_addr = BNO055_I2C_ADDR1;

	rslt = bno055_init(&bno055);
	if(rslt != BNO055_SUCCESS)
		return rslt;

	rslt = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	if(rslt != BNO055_SUCCESS)
		return rslt;

	rslt = bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
	if(rslt != BNO055_SUCCESS)
		return rslt;

	rslt = bno055_set_accel_range(BNO055_ACCEL_RANGE_16G);

	return rslt;
}

int8_t fetch_bme()
{
	int8_t rslt;
	static uint8_t cntr = 0;

	rslt = bme280_get_sensor_data(BME280_ALL, &bme_data, &bme);
	if (!rslt)
	{
		if (!bme_calibrated) {
			bme_basepressure += bme_data.pressure/100;
			bme_calib_counter++;

			if (bme_calib_counter == BARO_CALIB_N) {
				bme_basepressure /= BARO_CALIB_N;
				bme_calibrated = true;
			}
		} else if (cntr==0) {
			can_setFrame(bme_basepressure, DATA_ID_CALIB_PRESSURE, HAL_GetTick());
		}

		can_setFrame(bme_data.temperature, DATA_ID_TEMPERATURE, HAL_GetTick());
		can_setFrame(bme_data.pressure/100, DATA_ID_PRESSURE, HAL_GetTick());
		if(!cntr)
		{
			sprintf(buf, "Pres: %"PRIu32"\nTemp: %"PRIu32"\nHum: %"PRIu32"\n",
					bme_data.pressure, bme_data.temperature, bme_data.humidity);
			//INFO(buf);
		}
	}

	cntr = ++cntr < 30 ? cntr : 0;

	return rslt;
}

int8_t fetch_bno()
{
	int8_t rslt = 0;
	static uint8_t cntr = 0;

	rslt += bno055_convert_float_accel_xyz_mg (&accel);
	rslt += bno055_convert_float_mag_xyz_uT (&mag);
	rslt += bno055_convert_float_gyro_xyz_rps (&gyro);

	if(!rslt)
	{
		can_setFrame((int32_t) accel.x, DATA_ID_ACCELERATION_X, HAL_GetTick());
		can_setFrame((int32_t) accel.y, DATA_ID_ACCELERATION_Y, HAL_GetTick());
		can_setFrame((int32_t) accel.z, DATA_ID_ACCELERATION_Z, HAL_GetTick());
		can_setFrame((int32_t)(1000*gyro.x), DATA_ID_GYRO_X, HAL_GetTick());
		can_setFrame((int32_t)(1000*gyro.y), DATA_ID_GYRO_Y, HAL_GetTick());
		can_setFrame((int32_t)(1000*gyro.z), DATA_ID_GYRO_Z, HAL_GetTick());
		if(!cntr)
		{
			sprintf(buf, "Accel: [%f, %f, %f]\n", accel.x, accel.y, accel.z);
			//INFO(buf);
			sprintf(buf, "Gyro: [%f, %f, %f]\n", gyro.x, gyro.y, gyro.z);
			//INFO(buf);
			sprintf(buf, "Mag: [%f, %f, %f]\n\n\n", mag.x, mag.y, mag.z);
			//INFO(buf);
		}
	}

	cntr = ++cntr < 10 ? cntr : 0;

	return rslt;
}

int8_t stm32_i2c_read (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	vTaskSuspendAll();
	uint8_t rslt = HAL_I2C_Mem_Read(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	xTaskResumeAll();
	return rslt;
}

int8_t stm32_i2c_write (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	vTaskSuspendAll();
	uint8_t rslt = HAL_I2C_Mem_Write(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	xTaskResumeAll();
	return rslt;
}

void stm32_delay_ms (uint32_t delay)
{
	osDelay(delay);
}
