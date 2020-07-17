/**
  ******************************************************************************
  * File Name          : sensor_board.c
  * Description        : This file provides code for the configuration
  *                      of the sensor board.
  * Author 			   : Delfosse Quentin
  * Date    		   : Feb 2020

  * Notes : The code below was developed to implement a redundancy on the 4 sensors
  * 		used by this board, but can still work with the previous versions of the
  * 		sensor boards.

  ******************************************************************************
  */


#include <can_transmission.h>
#include "../../../HostBoard/Inc/Sensors/sensor_board.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include "../../../HostBoard/Inc/debug/led.h"
#include "../../../HostBoard/Inc/debug/console.h"
#include "../../../HostBoard/Inc/Misc/Common.h"
#include "../../../HostBoard/Inc/Misc/rocket_constants.h"
#include "../../../HostBoard/Inc/Sensors/BME280/bme280.h"
#include "../../../HostBoard/Inc/Sensors/BNO055/bno055.h"

#define I2C_TIMEOUT 3
#define FMPI2C_TIMEOUT 3
#define BARO_CALIB_N 128
#define normal_coef 3.000   // coefficient for a 99 % confidence interval
#define MAX_SENSOR_NUMBER 4

/* sensor_id is the index of the sensor, which is different form the dev_id ( defined in bme280_dev)
 * or dev_addr ( defined in bno055_t) representing its address for the I2C protocol
 */

int8_t init_bme(uint8_t sensor_id, int8_t rslt_bme[MAX_SENSOR_NUMBER]);
int8_t init_bno(uint8_t sensor_id, int8_t rslt_bno[MAX_SENSOR_NUMBER]);
int8_t fetch_bme(uint8_t sensor_id, int8_t rslt_bme[MAX_SENSOR_NUMBER]);
int8_t fetch_bno(uint8_t sensor_id, int8_t rslt_bno[MAX_SENSOR_NUMBER]);

int8_t stm32_i2c_read (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_i2c_write (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void stm32_delay_ms (uint32_t delay);
void sensor_elimination_4(float value_0, float value_1, float value_2, float value_3, float *correct_value, bool erroneous_sensor[MAX_SENSOR_NUMBER]);
void sensor_elimination_3(float value[MAX_SENSOR_NUMBER], uint8_t index_0, uint8_t index_1, uint8_t index_2, float *correct_value, bool erroneous_sensor[MAX_SENSOR_NUMBER]);
void sensor_elimination_2(float value[MAX_SENSOR_NUMBER], uint8_t index_0, uint8_t index_1, float *correct_value, bool erroneous_sensor[MAX_SENSOR_NUMBER]);
bool within_conf_interval_4(float value_0, float data_1, float data_2, float data_3);
bool within_conf_interval_3(float value_0, float data_1, float data_2);
void bme_redundancy(int8_t rslt_bme[MAX_SENSOR_NUMBER]);
void bno_redundancy(int8_t rslt_bno[MAX_SENSOR_NUMBER]);
void bme_data_process(uint8_t bme_init[MAX_SENSOR_NUMBER], int8_t rslt_bme[MAX_SENSOR_NUMBER], uint8_t cntr);
void bno_data_process(uint8_t imu_init[MAX_SENSOR_NUMBER], int8_t rslt_bno[MAX_SENSOR_NUMBER], uint8_t cntr);

extern I2C_HandleTypeDef hi2c3;
extern FMPI2C_HandleTypeDef hfmpi2c1;

char buf[300];
uint8_t current_sensor = 0;

struct bno055_data
{
	struct bno055_accel_float_t accel;
	struct bno055_gyro_float_t gyro;
	struct bno055_mag_float_t mag;
};

struct bme280_data_float
{
	float temperature;
	float pressure;
	float basepressure;
};

//## BME280 ## Barometer
struct bme280_dev bme[MAX_SENSOR_NUMBER];
struct bme280_data bme_data[MAX_SENSOR_NUMBER];
struct bme280_data_float bme_data_float[MAX_SENSOR_NUMBER];
struct bme280_data_float correct_bme_data;
//float correct_bme_basepressure;
uint32_t bme_calib_counter[MAX_SENSOR_NUMBER] = {};
//uint32_t correct_bme_calib_counter = 0;
bool bme_calibrated[MAX_SENSOR_NUMBER] = {false};
//bool correct_bme_calibrated = {false};

//## BNO055 ## IMU
struct bno055_t bno[MAX_SENSOR_NUMBER];
struct bno055_data bno_data[MAX_SENSOR_NUMBER];
struct bno055_data correct_bno_data;

uint8_t led_sensor_id_imu, led_sensor_id_baro;
uint8_t set_sensor_led(uint8_t id, uint8_t flag) {
	if (flag) { // success
		led_set_TK_rgb(id, 0, 50, 0);
	} else { // fail
		led_set_TK_rgb(id, 500, 0, 0);
	}
	return flag;
}


/*
 * Main function used to initialize and fetch the sensors.
 *
 * If at least one BME or BNO has been initialized, the redundancy
 * algorithm is called.
 */

uint32_t debug_counter = 0;


void TK_sensor_board(void const * argument) {

	uint8_t imu_init[MAX_SENSOR_NUMBER] = {}, baro_init[MAX_SENSOR_NUMBER]= {};
	osDelay(500);
	uint8_t cntr = 0;
	int8_t rslt_bme[MAX_SENSOR_NUMBER] = {1,1,1,1}, rslt_bno[MAX_SENSOR_NUMBER] = {1,1,1,1}; // rslt_xxx = 0 if no problem, 1 otherwise

	led_sensor_id_imu  = led_register_TK();
	led_sensor_id_baro = led_register_TK();

	for(;;) {
		if (imu_init[0]) { //BNO
			set_sensor_led(led_sensor_id_imu, fetch_bno(0, rslt_bno) == BNO055_SUCCESS); //BNO055_SUCCESS = 0
		} else {
			imu_init[0] = set_sensor_led(led_sensor_id_imu, init_bno(0, rslt_bno) == BNO055_SUCCESS);
		}
		if (imu_init[1]) {
			set_sensor_led(led_sensor_id_imu, fetch_bno(1, rslt_bno) == BNO055_SUCCESS);
		} else {
			imu_init[1] = set_sensor_led(led_sensor_id_imu, init_bno(1, rslt_bno) == BNO055_SUCCESS);
		}
		if (imu_init[2]) {
			set_sensor_led(led_sensor_id_imu, fetch_bno(2, rslt_bno) == BNO055_SUCCESS);
		} else {
			imu_init[2] = set_sensor_led(led_sensor_id_imu, init_bno(2, rslt_bno) == BNO055_SUCCESS);
		}
		if (imu_init[3]) {
			set_sensor_led(led_sensor_id_imu, fetch_bno(3, rslt_bno) == BNO055_SUCCESS);
		} else {
			imu_init[3] = set_sensor_led(led_sensor_id_imu, init_bno(3, rslt_bno) == BNO055_SUCCESS);
		}
		if (baro_init[0]) { //BME
			set_sensor_led(led_sensor_id_baro, fetch_bme(0, rslt_bme) == BME280_OK); //BME280_OK = 0
		} else {
			baro_init[0] = set_sensor_led(led_sensor_id_baro, init_bme(0, rslt_bme) == BME280_OK);
		}
		if (baro_init[1]) {
			set_sensor_led(led_sensor_id_baro, fetch_bme(1, rslt_bme) == BME280_OK);
		} else {
			baro_init[1] = set_sensor_led(led_sensor_id_baro, init_bme(1, rslt_bme) == BME280_OK);
		}
		if (baro_init[2]) {
			set_sensor_led(led_sensor_id_baro, fetch_bme(2, rslt_bme) == BME280_OK);
		} else {
			baro_init[2] = set_sensor_led(led_sensor_id_baro, init_bme(2, rslt_bme) == BME280_OK);
		}
		if (baro_init[3]) {
			set_sensor_led(led_sensor_id_baro, fetch_bme(3, rslt_bme) == BME280_OK);
		} else {
			baro_init[3] = set_sensor_led(led_sensor_id_baro, init_bme(3, rslt_bme) == BME280_OK);
		}

		osDelay(10);

		if(baro_init[0] || baro_init[1] || baro_init[2] || baro_init[3]
		    || imu_init[0] || imu_init[1] || imu_init[2] || imu_init[3])
		{ // Redundancy
			bme_data_process(baro_init, rslt_bme, cntr);
			bno_data_process(imu_init, rslt_bno, cntr);
		}

		if(debug_counter++ % 100 == 0) {
			rocket_log("\nActive sensors\n");
			if(baro_init[0]) rocket_log("Barometer 0\n");
			if(baro_init[1]) rocket_log("Barometer 1\n");
			if(baro_init[2]) rocket_log("Barometer 2\n");
			if(baro_init[3]) rocket_log("Barometer 3\n");
			if(imu_init[0]) rocket_log("IMU 0\n");
			if(imu_init[1]) rocket_log("IMU 1\n");
			if(imu_init[2]) rocket_log("IMU 2\n");
			if(imu_init[3]) rocket_log("IMU 3\n");
			rocket_log("\n");
		}

		cntr = ++cntr < 30 ? cntr : 2;
	}
}

/*
 * Initialization functions :
 *
 * Used to initialize the sensors
 * Returns an array ( rslt_bme, rslt_bno ) of size MAX_SENSOR_NUMBER
 * stating if the initialization was completed or not
 *
 * - rslt_XXX = 1 => failed to initialize
 * - rslt_XXX = 0 => initialization complete
 *
 */


int8_t init_bme(uint8_t sensor_id, int8_t rslt_bme[MAX_SENSOR_NUMBER])
{
	current_sensor = sensor_id;
	if (sensor_id == 1 || sensor_id == 2) {
		bme[sensor_id].dev_id = BME280_I2C_ADDR_PRIM;
	}
	else if (sensor_id == 0 || sensor_id == 3) {
		bme[sensor_id].dev_id = BME280_I2C_ADDR_SEC;
	}
	bme[sensor_id].intf = BME280_I2C_INTF;
	bme[sensor_id].read = &stm32_i2c_read;
	bme[sensor_id].write = &stm32_i2c_write;
	bme[sensor_id].delay_ms = &stm32_delay_ms;

	rslt_bme[sensor_id] = bme280_init(&bme[sensor_id]); // Returns 1 if error
	if (rslt_bme[sensor_id] != BME280_OK) {
		return rslt_bme[sensor_id];
	}
	//Always read the current settings before writing
	rslt_bme[sensor_id] = bme280_get_sensor_settings(&bme[sensor_id]);
	if (rslt_bme[sensor_id] != BME280_OK) {
		return rslt_bme[sensor_id];
	}
	//Overwrite the desired settings
	bme[sensor_id].settings.filter = BME280_FILTER_COEFF_OFF;
	bme[sensor_id].settings.osr_p = BME280_OVERSAMPLING_8X;
	bme[sensor_id].settings.osr_t = BME280_OVERSAMPLING_1X;
	bme[sensor_id].settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

	rslt_bme[sensor_id] = bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &bme[sensor_id]);
	if (rslt_bme[sensor_id] != BME280_OK) {
		return rslt_bme[sensor_id];
	}
	//Always set the power mode after setting the configuration
	rslt_bme[sensor_id] = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme[sensor_id]);

	bme_calibrated[sensor_id] = false;
	bme_calib_counter[sensor_id] = 0;
	bme_data_float[sensor_id].basepressure = 0;
	return rslt_bme[sensor_id];
}

int8_t init_bno(uint8_t sensor_id, int8_t rslt_bno[MAX_SENSOR_NUMBER])
{
	current_sensor = sensor_id;
	if (sensor_id == 1 || sensor_id == 2) {
		bno[sensor_id].dev_addr = BNO055_I2C_ADDR1;
	}
	else if (sensor_id == 0 || sensor_id == 3) {
		bno[sensor_id].dev_addr = BNO055_I2C_ADDR2;
	}
	bno[sensor_id].bus_write = &stm32_i2c_write;
	bno[sensor_id].bus_read = &stm32_i2c_read;
	bno[sensor_id].delay_msec = &stm32_delay_ms;

	rslt_bno[sensor_id] = bno055_init(&bno[sensor_id]); // Returns 1 if error
	if(rslt_bno[sensor_id] != BNO055_SUCCESS)
		return rslt_bno[sensor_id];

	rslt_bno[sensor_id] = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	if(rslt_bno[sensor_id] != BNO055_SUCCESS)
		return rslt_bno[sensor_id];

	rslt_bno[sensor_id] = bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
	if(rslt_bno[sensor_id] != BNO055_SUCCESS)
		return rslt_bno[sensor_id];

	rslt_bno[sensor_id] = bno055_set_accel_range(BNO055_ACCEL_RANGE_16G);

	return rslt_bno[sensor_id];
}

/*
 * Fetch functions :
 *
 * Used to fetch the sensors
 * Returns an array ( rslt_bme, rslt_bno ) of size MAX_SENSOR_NUMBER
 * stating if the fetch was completed or not
 *s
 * - rslt_XXX = 1 => failed to fetch
 * - rslt_XXX = 0 => fetch complete
 *
 */

int8_t fetch_bme(uint8_t sensor_id, int8_t rslt_bme[MAX_SENSOR_NUMBER])
{
	current_sensor = sensor_id;
	static uint8_t cntr = 0;
	bme[sensor_id].read = &stm32_i2c_read;
	bme[sensor_id].write = &stm32_i2c_write;

	rslt_bme[sensor_id] = bme280_get_sensor_data(BME280_ALL, &bme_data[sensor_id], &bme[sensor_id]);
	bme_data_float[sensor_id].temperature = (float) bme_data[sensor_id].temperature;
	bme_data_float[sensor_id].pressure = (float) bme_data[sensor_id].pressure/100;
	if (!rslt_bme[sensor_id])
	{
		if (!bme_calibrated[sensor_id]) {
			bme_data_float[sensor_id].basepressure += (float) bme_data[sensor_id].pressure/100;
			bme_calib_counter[sensor_id]++;

			if (bme_calib_counter[sensor_id] == BARO_CALIB_N) {
				bme_data_float[sensor_id].basepressure /= BARO_CALIB_N;
				bme_calibrated[sensor_id] = true;
			}
		} else if (cntr==0) {
			can_setFrame(bme_data_float[sensor_id].basepressure, DATA_ID_CALIB_PRESSURE, HAL_GetTick());
		}

		/*can_setFrame(bme_data_float[sensor_id].temperature, DATA_ID_TEMPERATURE, HAL_GetTick());
		can_setFrame(bme_data_float[sensor_id].pressure/100, DATA_ID_PRESSURE, HAL_GetTick());*/

		if(!cntr)
		{
			/*sprintf(buf, "Pres: %"PRIu32"\nTemp: %"PRIu32"\nHum: %"PRIu32"\n",
					bme_data_float[sensor_id].pressure, bme_data_float[sensor_id].temperature, bme_data[sensor_id].humidity);*/
			//INFO(buf);
		}
	}

	cntr = ++cntr < 30 ? cntr : 0;

	return rslt_bme[sensor_id];
}

int8_t fetch_bno(uint8_t sensor_id, int8_t rslt_bno[MAX_SENSOR_NUMBER])
{
	current_sensor = sensor_id;
	static uint8_t cntr = 0;
	bno[sensor_id].bus_write = &stm32_i2c_write;
	bno[sensor_id].bus_read = &stm32_i2c_read;

	rslt_bno[sensor_id] += bno055_convert_float_accel_xyz_mg (&bno_data[sensor_id].accel);
	rslt_bno[sensor_id] += bno055_convert_float_mag_xyz_uT (&bno_data[sensor_id].mag);
	rslt_bno[sensor_id] += bno055_convert_float_gyro_xyz_rps (&bno_data[sensor_id].gyro);

	if(!rslt_bno[sensor_id])
	{
		/*can_setFrame((int32_t) bno_data[sensor_id].accel.x, DATA_ID_ACCELERATION_X, HAL_GetTick());
		can_setFrame((int32_t) bno_data[sensor_id].accel.y, DATA_ID_ACCELERATION_Y, HAL_GetTick());
		can_setFrame((int32_t) bno_data[sensor_id].accel.z, DATA_ID_ACCELERATION_Z, HAL_GetTick());
		can_setFrame((int32_t)(1000*bno_data[sensor_id].gyro.x), DATA_ID_GYRO_X, HAL_GetTick());
		can_setFrame((int32_t)(1000*bno_data[sensor_id].gyro.y), DATA_ID_GYRO_Y, HAL_GetTick());
		can_setFrame((int32_t)(1000*bno_data[sensor_id].gyro.z), DATA_ID_GYRO_Z, HAL_GetTick());*/
		if(!cntr)
		{
			//sprintf(buf, "Accel: [%f, %f, %f]\n", bno_data[sensor_id].accel.x, bno_data[sensor_id].accel.y, bno_data[sensor_id].accel.z);
			//INFO(buf);
			//sprintf(buf, "Gyro: [%f, %f, %f]\n", bno_data[sensor_id].gyro.x, bno_data[sensor_id].gyro.y, bno_data[sensor_id].gyro.z);
			//INFO(buf);
			//sprintf(buf, "Mag: [%f, %f, %f]\n\n\n", bno_data[sensor_id].mag.x, bno_data[sensor_id].mag.y, bno_data[sensor_id].mag.z);
			//INFO(buf);
		}
	}

	cntr = ++cntr < 10 ? cntr : 0;

	return rslt_bno[sensor_id];
}

/*
 * stm32_i2c functions :
 *
 * I2C3 : Sensor 0 and Sensor 1
 * FMPI2C1 : Sensor 2 and Sensor 3
 *
 */

int8_t stm32_i2c_read (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	vTaskSuspendAll();
	int8_t rslt = 0;
	if (current_sensor == 0 || current_sensor == 1) {
		rslt = HAL_I2C_Mem_Read(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	}
	else if (current_sensor == 2 || current_sensor == 3) {
		rslt = HAL_FMPI2C_Mem_Read(&hfmpi2c1, dev_id << 1, reg_addr, FMPI2C_MEMADD_SIZE_8BIT, data, len, FMPI2C_TIMEOUT);
	}
	xTaskResumeAll();
	return rslt;
}

int8_t stm32_i2c_write (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	vTaskSuspendAll();
	int8_t rslt = 0;
	if (current_sensor == 0 || current_sensor == 1) {
		rslt = HAL_I2C_Mem_Write(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	}
	else if (current_sensor == 2 || current_sensor == 3) {
		rslt = HAL_FMPI2C_Mem_Write(&hfmpi2c1, dev_id << 1, reg_addr, FMPI2C_MEMADD_SIZE_8BIT, data, len, FMPI2C_TIMEOUT);
	}
	xTaskResumeAll();
	return rslt;
}

void stm32_delay_ms (uint32_t delay)
{
	osDelay(delay);
}

/*
 * sensor_elimination :
 *
 * Used to compare the data from the different sensors
 *
 * Input Arguments :
 *  - values from the different sensors
 *  - correct value determined by the algorithm
 *  - array of size MAX_SENSOR_NUMBER to know if there was an initialization or
 *    fetch error
 *
 * If sensor_elimination_4 finds an erroneous sensor, its values are not taken into
 * account and sensor_elimination_3 takes over. Indeed, the intervals of confidence
 * calculated by sensor_elimination _4 might have been corrupted by the erroneous sensor,
 * therefore new intervals have to be calculated.
 *
 */

void sensor_elimination_4(float value_0, float value_1, float value_2, float value_3, float *correct_value, bool erroneous_sensor[MAX_SENSOR_NUMBER]) {
	float value[MAX_SENSOR_NUMBER] = {value_0, value_1, value_2, value_3};
	if (!erroneous_sensor[0]) { // Checking if one of the sensor has been eliminated before
		if (!erroneous_sensor[1]) {
			if (!erroneous_sensor[2]) {
				if (!erroneous_sensor[3]) {
					if (within_conf_interval_4(value_0, value_1, value_2, value_3)) { // If all sensors are working properly so far, we check the confidence intervals
						if (within_conf_interval_4(value_1, value_0, value_2, value_3)) {
							if (within_conf_interval_4(value_2, value_0, value_1, value_3)) {
								if (within_conf_interval_4(value_3, value_1, value_2, value_0)) {
									*correct_value = (value_0 + value_1 + value_2 + value_3)/MAX_SENSOR_NUMBER;
								}
								else { // Value_3 out, redundancy on the 3 sensors left
									sensor_elimination_3(value, 0, 1, 2, correct_value, erroneous_sensor);
								}
							}
							else { // Value_2 out
								sensor_elimination_3(value, 0, 1, 3, correct_value, erroneous_sensor);
							}
						}
						else { // value_1 out
							sensor_elimination_3(value, 0, 2, 3, correct_value, erroneous_sensor);
						}
					}
					else { // value_0 out
						sensor_elimination_3(value, 1, 2, 3, correct_value, erroneous_sensor);
					}
				}
				else { // Value_3 not fetched, redundancy on the 3 sensors left
					sensor_elimination_3(value, 0, 1, 2, correct_value, erroneous_sensor);
				}
			}
			else { // Value_2 not fetched
				sensor_elimination_3(value, 0, 1, 3, correct_value, erroneous_sensor);
			}
		}
		else { // value_1 not fetched
			sensor_elimination_3(value, 0, 2, 3, correct_value, erroneous_sensor);
		}
	}
	else { // value_0 not fetched
		sensor_elimination_3(value, 1, 2, 3, correct_value, erroneous_sensor);
	}
}

void sensor_elimination_3(float value[MAX_SENSOR_NUMBER], uint8_t index_0, uint8_t index_1, uint8_t index_2, float *correct_value, bool erroneous_sensor[MAX_SENSOR_NUMBER]) {
	if (!erroneous_sensor[index_0]) { // Checking if one of the sensor has been eliminated before
		if (!erroneous_sensor[index_1]) {
			if (!erroneous_sensor[index_2]) {
				if (within_conf_interval_3(value[index_0], value[index_1], value[index_2])) { // If all sensors are working properly so far, we check the confidence intervals
					if (within_conf_interval_3(value[index_1], value[index_0], value[index_2])) {
						if (within_conf_interval_3(value[index_2], value[index_1], value[index_0])) {
							*correct_value = (value[index_0] + value[index_1] + value[index_2])/(MAX_SENSOR_NUMBER-1);
						}
						else { // Value_2 out, no redundancy on the 2 sensors left
							*correct_value = (value[index_0] + value[index_1])/(MAX_SENSOR_NUMBER-2);
						}
					}
					else { // Value_1 out
						*correct_value = (value[index_0] + value[index_2])/(MAX_SENSOR_NUMBER-2);
					}
				}
				else { // Value_0 out
					*correct_value = (value[index_1] + value[index_2])/(MAX_SENSOR_NUMBER-2);
				}
			}
			else { // Value_2 not fetched
				sensor_elimination_2(value, index_0, index_1, correct_value, erroneous_sensor);
			}
		}
		else { // Value_1 not fetched
			sensor_elimination_2(value, index_0, index_2, correct_value, erroneous_sensor);
		}
	}
	else { // Value_0 not fetched
		sensor_elimination_2(value, index_1, index_2, correct_value, erroneous_sensor);
	}
}

void sensor_elimination_2(float value[MAX_SENSOR_NUMBER], uint8_t index_0, uint8_t index_1, float *correct_value, bool erroneous_sensor[MAX_SENSOR_NUMBER]) {
	if (!erroneous_sensor[index_0]) { // Checking if one of the sensor has been eliminated before
		if (!erroneous_sensor[index_1]) {
			*correct_value = (value[index_0] + value[index_1])/(MAX_SENSOR_NUMBER-2);
		}
	}
	else {
		if (!erroneous_sensor[index_1]) {
			*correct_value = value[index_1]/(MAX_SENSOR_NUMBER-3);
		}
		else { } // If no sensors are fetched, correct_value doesn't change
	}
}

/*
 * within_conf_interval
 *
 * Checks if the FIRST GIVEN ARGUMENT is included inside [-3*sigma;+3*sigma]
 * of the normal distribution for a 99% confidence interval
 * The normal distribution is baised on the values of the two or three other
 * sensors
 *
 */


bool within_conf_interval_4(float data_0, float data_1, float data_2, float data_3) {
	if (data_1 != data_2 && data_1 != data_3 && data_2 != data_3) {
		float mean = (data_1 + data_2 + data_3)/(MAX_SENSOR_NUMBER-1);
		float std_dev = sqrt( pow(data_1 - mean,2) + pow(data_2 - mean,2) + pow(data_3 - mean,2) );
		float normal_data_0 = (data_0 - mean)/std_dev;
		return ( -normal_coef < normal_data_0 && normal_coef > normal_data_0 );
	}
	else { return false; }
}

bool within_conf_interval_3(float data_0, float data_1, float data_2) {
	if (data_1 != data_2) { // If two values are equal, returns their shared value
		float mean = (data_1 + data_2)/(MAX_SENSOR_NUMBER-2);
		float std_dev = sqrt( pow(data_1 - mean,2) + pow(data_2 - mean,2) );
		float normal_data_0 = (data_0 - mean)/std_dev;
		return ( -normal_coef < normal_data_0 && normal_coef > normal_data_0 );
	}
	else { return false; }
}

/*
 * XXX_redundancy
 *
 * Calls the redundancy algorithm for each kind of data given by the sensors
 *
 */

void bno_redundancy(int8_t rslt_bno[MAX_SENSOR_NUMBER]) {
	bool erroneous_bno[MAX_SENSOR_NUMBER] = {false};
	for (uint8_t i = 0; i<MAX_SENSOR_NUMBER; i++) {
		erroneous_bno[i] = rslt_bno[i]; // If the sensor hasn't been fetched, we can't have a rogue value
	}
	sensor_elimination_4(bno_data[0].accel.x, bno_data[1].accel.x, bno_data[2].accel.x, bno_data[3].accel.x, &correct_bno_data.accel.x, erroneous_bno); // Attention ! Il faut toujours que les baros soient appelés par ordre d'indice croissant !
	sensor_elimination_4(bno_data[0].accel.y, bno_data[1].accel.y, bno_data[2].accel.y, bno_data[3].accel.y, &correct_bno_data.accel.y, erroneous_bno);
	sensor_elimination_4(bno_data[0].accel.z, bno_data[1].accel.z, bno_data[2].accel.z, bno_data[3].accel.z, &correct_bno_data.accel.z, erroneous_bno);
	sensor_elimination_4(bno_data[0].gyro.x, bno_data[1].gyro.x, bno_data[2].gyro.x, bno_data[3].gyro.x, &correct_bno_data.gyro.x, erroneous_bno);
	sensor_elimination_4(bno_data[0].gyro.y, bno_data[1].gyro.y, bno_data[2].gyro.y, bno_data[3].gyro.y, &correct_bno_data.gyro.y, erroneous_bno);
	sensor_elimination_4(bno_data[0].gyro.z, bno_data[1].gyro.z, bno_data[2].gyro.z, bno_data[3].gyro.z, &correct_bno_data.gyro.z, erroneous_bno);
}

void bme_redundancy(int8_t rslt_bme[MAX_SENSOR_NUMBER]) {
	bool erroneous_bme[MAX_SENSOR_NUMBER] = {false};
	for (uint8_t i = 0; i<MAX_SENSOR_NUMBER; i++) {
			erroneous_bme[i] = rslt_bme[i];
	}
	sensor_elimination_4(bme_data_float[0].pressure, bme_data_float[1].pressure, bme_data_float[2].pressure, bme_data_float[3].pressure, &correct_bme_data.pressure, erroneous_bme);
	sensor_elimination_4(bme_data_float[0].temperature, bme_data_float[1].temperature, bme_data_float[2].temperature, bme_data_float[3].temperature, &correct_bme_data.temperature,erroneous_bme);
}

/*
 * XXX_data_process
 *
 * Used to send the sensors data through the can, whether there was a redundancy or not.
 * No redundancy means that only one sensor has been initialized/fetched, its values are then sent.
 *
 */


void bme_data_process(uint8_t baro_init[MAX_SENSOR_NUMBER], int8_t rslt_bme[MAX_SENSOR_NUMBER], uint8_t cntr)
{
	if ( baro_init[0] || baro_init[1] || baro_init[2] || baro_init[3] )
	{ // Checks if at least one BME is initialized
		if(cntr>0)
		{
			if ( (!rslt_bme[0] && !rslt_bme[1]) || (!rslt_bme[1] && !rslt_bme[2]) || (!rslt_bme[0] && !rslt_bme[2]) ||
				 (!rslt_bme[0] && !rslt_bme[3]) || (!rslt_bme[1] && !rslt_bme[3]) || (!rslt_bme[2] && !rslt_bme[3]))
			{ // Checks if at least two barometers have correctly been fetched

				bme_redundancy(rslt_bme);
				/*if (!correct_bme_calibrated)
				{
					correct_bme_basepressure += correct_bme_data.pressure/100;
					correct_bme_calib_counter++;
					if (correct_bme_calib_counter == BARO_CALIB_N)
					{
						correct_bme_basepressure /= BARO_CALIB_N;
						correct_bme_calibrated = true;
					}
				}
				if (cntr==1)
				{
						can_setFrame(correct_bme_basepressure, DATA_ID_CALIB_PRESSURE, HAL_GetTick());
				}*/
				can_setFrame(correct_bme_data.temperature, DATA_ID_TEMPERATURE, HAL_GetTick());
				can_setFrame(correct_bme_data.pressure/100, DATA_ID_PRESSURE, HAL_GetTick());
			}

			else { // No BME redundancy
				for (uint8_t i = 0; i<3; i++) {
					if (!rslt_bme[i])
					{
						/*if (!correct_bme_calibrated)
						{
							correct_bme_basepressure += bme_data_float[i].pressure/100;
							correct_bme_calib_counter++;
							if (correct_bme_calib_counter == BARO_CALIB_N)
							{
								correct_bme_basepressure /= BARO_CALIB_N;
								correct_bme_calibrated = true;
							}
						}
						if (cntr==1)
						{
							can_setFrame(correct_bme_basepressure, DATA_ID_CALIB_PRESSURE, HAL_GetTick()); // Still needs the ID of the correct data for the CanBus to recognize it
						}*/
						can_setFrame(bme_data_float[i].temperature, DATA_ID_TEMPERATURE, HAL_GetTick());
						can_setFrame(bme_data_float[i].pressure/100, DATA_ID_PRESSURE, HAL_GetTick());
					}
				}
			}
		}
	}
}

void bno_data_process(uint8_t imu_init[MAX_SENSOR_NUMBER], int8_t rslt_bno[MAX_SENSOR_NUMBER], uint8_t cntr)
{
	if ( imu_init[0] || imu_init[1] || imu_init[2] || imu_init[3] )
	{ // Checks if at least one BNO is initialized
		if(cntr>0)
		{
			if ( (!rslt_bno[0] && !rslt_bno[1]) || (!rslt_bno[1] && !rslt_bno[2]) || (!rslt_bno[0] && !rslt_bno[2]) ||
				 (!rslt_bno[0] && !rslt_bno[3]) || (!rslt_bno[1] && !rslt_bno[3]) || (!rslt_bno[2] && !rslt_bno[3]))
			{ // Checks if at least two IMU have correctly been fetched

				bno_redundancy(rslt_bno);
				can_setFrame((int32_t) correct_bno_data.accel.x, DATA_ID_ACCELERATION_X, HAL_GetTick());
				can_setFrame((int32_t) correct_bno_data.accel.y, DATA_ID_ACCELERATION_Y, HAL_GetTick());
				can_setFrame((int32_t) correct_bno_data.accel.z, DATA_ID_ACCELERATION_Z, HAL_GetTick());
				can_setFrame((int32_t)(1000*correct_bno_data.gyro.x), DATA_ID_GYRO_X, HAL_GetTick());
				can_setFrame((int32_t)(1000*correct_bno_data.gyro.y), DATA_ID_GYRO_Y, HAL_GetTick());
				can_setFrame((int32_t)(1000*correct_bno_data.gyro.z), DATA_ID_GYRO_Z, HAL_GetTick());
			}

			else
			{ // No BNO redundancy
				for (uint8_t i = 0; i<3; i++)
				{
					if (!rslt_bno[i])
					{
						can_setFrame((int32_t) bno_data[i].accel.x, DATA_ID_ACCELERATION_X, HAL_GetTick());
						can_setFrame((int32_t) bno_data[i].accel.y, DATA_ID_ACCELERATION_Y, HAL_GetTick());
						can_setFrame((int32_t) bno_data[i].accel.z, DATA_ID_ACCELERATION_Z, HAL_GetTick());
						can_setFrame((int32_t)(1000*bno_data[i].gyro.x), DATA_ID_GYRO_X, HAL_GetTick());
						can_setFrame((int32_t)(1000*bno_data[i].gyro.y), DATA_ID_GYRO_Y, HAL_GetTick());
						can_setFrame((int32_t)(1000*bno_data[i].gyro.z), DATA_ID_GYRO_Z, HAL_GetTick());
					}
				}
			}
		}
	}
}
