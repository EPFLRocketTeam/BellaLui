#include "../../../HostBoard/Inc/Sensors/sensor_board.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include "../../../HostBoard/Inc/CAN_communication.h"
#include "../../../HostBoard/Inc/led.h"
#include "../../../HostBoard/Inc/Misc/Common.h"
#include "../../../HostBoard/Inc/Misc/rocket_constants.h"
#include "../../../HostBoard/Inc/Sensors/BME280/bme280.h"
#include "../../../HostBoard/Inc/Sensors/BNO055/bno055.h"

#define I2C_TIMEOUT 3
#define BARO_CALIB_N 128
#define normal_coef 3.000   // coefficient for a 99 % confidence interval
#define MAX_SENSOR_NUMBER 3

int8_t init_bme(uint8_t sensor_id, uint8_t rslt_bme[MAX_SENSOR_NUMBER]); // sensor_id is the index of the sensor, which is different form the dev_id (bme) or dev_addr (bno) representing its address for the I2C protocol
int8_t init_bno(uint8_t sensor_id, uint8_t rslt_bno[MAX_SENSOR_NUMBER]);
int8_t fetch_bme(uint8_t sensor_id, uint8_t rslt_bme[MAX_SENSOR_NUMBER]);
int8_t fetch_bno(uint8_t sensor_id, uint8_t rslt_bno[MAX_SENSOR_NUMBER]);

int8_t stm32_i2c_read (uint8_t sensor_id, uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_i2c_write (uint8_t sensor_id, uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void stm32_delay_ms (uint32_t delay);
void sensor_elimination(double value_1, double value_2, double value_3, double correct_value, bool erroneous_sensor[MAX_SENSOR_NUMBER]);
bool within_conf_interval(double data_1, double data_2, double data_3);
void bme_redundancy(uint8_t rslt_bme[MAX_SENSOR_NUMBER]);
void bno_redundancy(uint8_t rslt_bno[MAX_SENSOR_NUMBER]);

extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hfmpi2c1;

struct bno055_data
{
	struct bno055_accel_float_t accel;
	struct bno055_gyro_float_t gyro;
	struct bno055_mag_float_t mag;
};

//## BME280 ##
struct bme280_dev bme[MAX_SENSOR_NUMBER];
struct bme280_data bme_data[MAX_SENSOR_NUMBER];
struct bme280_data correct_bme_data;
uint32_t bme_basepressure[MAX_SENSOR_NUMBER];
uint32_t correct_bme_basepressure;
uint32_t bme_calib_counter[MAX_SENSOR_NUMBER];
bool bme_calibrated[MAX_SENSOR_NUMBER] = {false};

//## BNO055 ##
struct bno055_t bno[MAX_SENSOR_NUMBER];
struct bno055_data bno_data[MAX_SENSOR_NUMBER];
struct bno055_data correct_bno_data;

int led_sensor_id_imu, led_sensor_id_baro;


int set_sensor_led(int id, int flag) {
	if (flag) { // success
		led_set_TK_rgb(id, 0, 50, 0);
	} else { // fail
		led_set_TK_rgb(id, 500, 0, 0);
	}
	return flag;
}


//Sensor Board thread
void TK_sensor_board(void const * argument) {

	int imu_init[MAX_SENSOR_NUMBER] = {0}, baro_init[MAX_SENSOR_NUMBER]= {0};
	osDelay(500);
	int cntr = 0;
	int8_t rslt_bme[MAX_SENSOR_NUMBER] = {1};
	int8_t rslt_bno[MAX_SENSOR_NUMBER] = {1};

	led_sensor_id_imu  = led_register_TK();
	led_sensor_id_baro = led_register_TK();

	for(;;) {
		if (imu_init[0]) {
			set_sensor_led(led_sensor_id_imu, fetch_bno(0, rslt_bno) == BNO055_SUCCESS);
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
		if (baro_init[0]) {
			set_sensor_led(led_sensor_id_baro, fetch_bme(0, rslt_bme) == BME280_OK);
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

		osDelay(10);

		if(!baro_init[0] && !baro_init[1] && !baro_init[2] && !imu_init[0] && !imu_init[1] && !imu_init[2]) { // If none of the sensors are initialized
			osDelay(1000);
		}

		else {
			if ( baro_init[0] || baro_init[1] || baro_init[2] ) { // Checks if at least one barometers is initialized
				if(cntr>0) {
					if ( (!rslt_bme[0] && !rslt_bme[1]) || (!rslt_bme[1] && !rslt_bme[2]) || (!rslt_bme[0] && !rslt_bme[2]) )  { // Checks if at least two barometers have correctly been fetched

						bme_redundancy(rslt_bme);
						if (cntr==1) {
							can_setFrame(correct_bme_basepressure, DATA_ID_CALIB_PRESSURE, HAL_GetTick());
						}
						can_setFrame(correct_bme_data.temperature, DATA_ID_TEMPERATURE, HAL_GetTick());
						can_setFrame(correct_bme_data.pressure/100, DATA_ID_PRESSURE, HAL_GetTick());
					}
					else {
						for (uint8_t i = 0; i<3; i++) {
							if (!rslt_bme[i]) {
								if (cntr==1) {
									can_setFrame(bme_basepressure[i], DATA_ID_CALIB_PRESSURE, HAL_GetTick()); // Still needs the ID of the correct data for the CanBus to recognize it
								}
								can_setFrame(bme_data[i].temperature, DATA_ID_TEMPERATURE, HAL_GetTick());
								can_setFrame(bme_data[i].pressure/100, DATA_ID_PRESSURE, HAL_GetTick());
							}
						}
					}
				}
			}

			if ( imu_init[0] || imu_init[1] || imu_init[2] ) { // Checks if at least one IMU is initialized
				if(cntr>0) {
					if ( (!rslt_bno[0] && !rslt_bno[1]) || (!rslt_bno[1] && !rslt_bno[2]) || (!rslt_bno[0] && !rslt_bno[2]) )  { // Checks if at least two IMU have correctly been fetched

						bno_redundancy(rslt_bno);
						can_setFrame((int32_t) correct_bno_data.accel.x, DATA_ID_ACCELERATION_X, HAL_GetTick());
						can_setFrame((int32_t) correct_bno_data.accel.y, DATA_ID_ACCELERATION_Y, HAL_GetTick());
						can_setFrame((int32_t) correct_bno_data.accel.z, DATA_ID_ACCELERATION_Z, HAL_GetTick());
						can_setFrame((int32_t)(1000*correct_bno_data.gyro.x), DATA_ID_GYRO_X, HAL_GetTick());
						can_setFrame((int32_t)(1000*correct_bno_data.gyro.y), DATA_ID_GYRO_Y, HAL_GetTick());
						can_setFrame((int32_t)(1000*correct_bno_data.gyro.z), DATA_ID_GYRO_Z, HAL_GetTick());
					}
					else {
						for (uint8_t i = 0; i<3; i++) {
							if (!rslt_bno[i]) {
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

		cntr = ++cntr < 30 ? cntr : 2;
	}
}

int8_t init_bme(uint8_t sensor_id, uint8_t rslt_bme[MAX_SENSOR_NUMBER])
{
	if (sensor_id == 0 || sensor_id == 2) {
		bme[sensor_id].dev_id = BME280_I2C_ADDR_PRIM;
	}
	else if (sensor_id == 1) {
		bme[sensor_id].dev_id = BME280_I2C_ADDR_SEC;
	}
	bme[sensor_id].intf = BME280_I2C_INTF;
	bme[sensor_id].read = &stm32_i2c_read;
	bme[sensor_id].write = &stm32_i2c_write;
	bme[sensor_id].delay_ms = &stm32_delay_ms;

	rslt_bme[sensor_id] = bme280_init(&bme[sensor_id]); // Returns 1 if error
	if(rslt_bme[sensor_id] != BME280_OK)
		return rslt_bme[sensor_id];

	//Always read the current settings before writing
	rslt_bme[sensor_id] = bme280_get_sensor_settings(&bme[sensor_id]);
	if(rslt_bme[sensor_id] != BME280_OK)
		return rslt_bme[sensor_id];

	//Overwrite the desired settings
	bme[sensor_id].settings.filter = BME280_FILTER_COEFF_OFF;
	bme[sensor_id].settings.osr_p = BME280_OVERSAMPLING_8X;
	bme[sensor_id].settings.osr_t = BME280_OVERSAMPLING_1X;
	bme[sensor_id].settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

	rslt_bme[sensor_id] = bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &bme[sensor_id]);
	if(rslt_bme[sensor_id] != BME280_OK)
		return rslt_bme[sensor_id];

	//Always set the power mode after setting the configuration
	rslt_bme[sensor_id] = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme[sensor_id]);

	bme_calibrated[sensor_id] = false;
	bme_calib_counter[sensor_id] = 0;
	bme_basepressure[sensor_id] = 0;
	return rslt_bme[sensor_id];
}

int8_t init_bno(uint8_t sensor_id, uint8_t rslt_bno[MAX_SENSOR_NUMBER])
{
	if (sensor_id == 0 || sensor_id == 2) {
		bno[sensor_id].dev_addr = BNO055_I2C_ADDR1;
	}
	else if (sensor_id == 1) {
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

int8_t fetch_bme(uint8_t sensor_id, uint8_t rslt_bme[MAX_SENSOR_NUMBER])
{
	static uint8_t cntr = 0;
	//bme[sensor_id].read = &stm32_i2c_read;
	//bme[sensor_id].write = &stm32_i2c_write;

	rslt_bme[sensor_id] = bme280_get_sensor_data(BME280_ALL, &bme_data[sensor_id], &bme[sensor_id]);
	if (!rslt_bme[sensor_id])
	{
		if (!bme_calibrated[sensor_id]) {
			bme_basepressure[sensor_id] += bme_data[sensor_id].pressure/100;
			bme_calib_counter[sensor_id]++;

			if (bme_calib_counter[sensor_id] == BARO_CALIB_N) {
				bme_basepressure[sensor_id] /= BARO_CALIB_N;
				bme_calibrated[sensor_id] = true;
			}
		} else if (cntr==0) {
			can_setFrame(bme_basepressure[sensor_id], DATA_ID_CALIB_PRESSURE, HAL_GetTick());
		}

		can_setFrame(bme_data[sensor_id].temperature, DATA_ID_TEMPERATURE, HAL_GetTick());
		can_setFrame(bme_data[sensor_id].pressure/100, DATA_ID_PRESSURE, HAL_GetTick());

		/*if(!cntr)
		{
			sprintf(buf, "Pres: %"PRIu32"\nTemp: %"PRIu32"\nHum: %"PRIu32"\n",
					bme_data[sensor_id].pressure, bme_data[sensor_id].temperature, bme_data[sensor_id].humidity);
			//INFO(buf);
		}*/
	}

	cntr = ++cntr < 30 ? cntr : 0;

	return rslt_bme[sensor_id];
}

int8_t fetch_bno(uint8_t sensor_id, uint8_t rslt_bno[MAX_SENSOR_NUMBER])
{
	static uint8_t cntr = 0;
	//bno[sensor_id].bus_write = &stm32_i2c_write;
	//bno[sensor_id].bus_read = &stm32_i2c_read;

	rslt_bno[sensor_id] += bno055_convert_float_accel_xyz_mg (&bno_data[sensor_id].accel);
	rslt_bno[sensor_id] += bno055_convert_float_mag_xyz_uT (&bno_data[sensor_id].mag);
	rslt_bno[sensor_id] += bno055_convert_float_gyro_xyz_rps (&bno_data[sensor_id].gyro);

	if(!rslt_bno[sensor_id])
	{
		can_setFrame((int32_t) bno_data[sensor_id].accel.x, DATA_ID_ACCELERATION_X, HAL_GetTick());
		can_setFrame((int32_t) bno_data[sensor_id].accel.y, DATA_ID_ACCELERATION_Y, HAL_GetTick());
		can_setFrame((int32_t) bno_data[sensor_id].accel.z, DATA_ID_ACCELERATION_Z, HAL_GetTick());
		can_setFrame((int32_t)(1000*bno_data[sensor_id].gyro.x), DATA_ID_GYRO_X, HAL_GetTick());
		can_setFrame((int32_t)(1000*bno_data[sensor_id].gyro.y), DATA_ID_GYRO_Y, HAL_GetTick());
		can_setFrame((int32_t)(1000*bno_data[sensor_id].gyro.z), DATA_ID_GYRO_Z, HAL_GetTick());
		/*if(!cntr)
		{
			sprintf(buf, "Accel: [%f, %f, %f]\n", bno_data[sensor_id].accel.x, bno_data[sensor_id].accel.y, bno_data[sensor_id].accel.z);
			//INFO(buf);
			sprintf(buf, "Gyro: [%f, %f, %f]\n", bno_data[sensor_id].gyro.x, bno_data[sensor_id].gyro.y, bno_data[sensor_id].gyro.z);
			//INFO(buf);
			sprintf(buf, "Mag: [%f, %f, %f]\n\n\n", bno_data[sensor_id].mag.x, bno_data[sensor_id].mag.y, bno_data[sensor_id].mag.z);
			//INFO(buf);
		}*/
	}

	cntr = ++cntr < 10 ? cntr : 0;

	return rslt_bno[sensor_id];
}

int8_t stm32_i2c_read (uint8_t sensor_id, uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	vTaskSuspendAll();
	uint8_t rslt = 0;
	if (sensor_id == 0 || sensor_id == 1) {
		rslt = HAL_I2C_Mem_Read(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	}
	else if (sensor_id == 2) {
		rslt = HAL_I2C_Mem_Read(&hfmpi2c1, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	}
	xTaskResumeAll();
	return rslt;
}

int8_t stm32_i2c_write (uint8_t sensor_id, uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	vTaskSuspendAll();
	uint8_t rslt = 0;
	if (sensor_id == 0 || sensor_id == 1) {
		rslt = HAL_I2C_Mem_Write(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	}
	else if (sensor_id == 2) {
		rslt = HAL_I2C_Mem_Write(&hfmpi2c1, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	}
	xTaskResumeAll();
	return rslt;
}

void stm32_delay_ms (uint32_t delay)
{
	osDelay(delay);
}


void sensor_elimination(double value_1, double value_2, double value_3, double correct_value, bool erroneous_sensor[MAX_SENSOR_NUMBER]) {
	if (!erroneous_sensor[0]) { // Checking if one of the sensor has been eliminated before
		if (!erroneous_sensor[1]) {
			if (!erroneous_sensor[2]) {
				if (within_conf_interval(value_1, value_2, value_3)) { // If all sensors are working properly so far, we check the confidence intervals
					if (within_conf_interval(value_2, value_1, value_3)) {
						if (within_conf_interval(value_3, value_1, value_2)) {
							correct_value = (value_1 + value_2 + value_3)/3;
						}
						else {
							erroneous_sensor[2] = true;
							correct_value = (value_1 + value_2)/2;
						}
					}
					else {
						erroneous_sensor[1] = true;
						correct_value = (value_1 + value_3)/2;
					}
				}
				else {
					erroneous_sensor[0] = true;
					correct_value = (value_2 + value_3)/2;
				}
			}
			else {
				correct_value = (value_1 + value_2)/2;
			}
		}
		else {
			correct_value = (value_1 + value_3)/2;
		}
	}
	else {
		correct_value = (value_2 + value_3)/2;
	}
}


bool within_conf_interval(double data_1, double data_2, double data_3) {
	if (data_2 != data_3) {
		double mean = (data_2 + data_3)/2;
		double std_dev = sqrt( pow(data_2 - mean,2) + pow(data_3 - mean,2) );
		double normal_data_1 = (data_1 - mean)/std_dev;
		return ( -normal_coef < normal_data_1 && normal_coef > normal_data_1 );
	}
	else { return false; }
}


void bno_redundancy(uint8_t rslt_bno[MAX_SENSOR_NUMBER]) {
	// Redundancy of bme
	bool erroneous_bno[MAX_SENSOR_NUMBER] = {false};
	for (uint8_t i = 0; i<3; i++) {
		erroneous_bno[i] = rslt_bno[i]; // If the sensor hasn't been fetched, we can't have a rogue value
	}
	sensor_elimination(bno_data[0].accel.x, bno_data[1].accel.x, bno_data[2].accel.x, correct_bno_data.accel.x, erroneous_bno); // Attention ! Il faut toujours que les baros soient appelés par ordre d'indice croissant !
	sensor_elimination(bno_data[0].accel.y, bno_data[1].accel.y, bno_data[2].accel.y, correct_bno_data.accel.y, erroneous_bno);
	sensor_elimination(bno_data[0].accel.z, bno_data[1].accel.z, bno_data[2].accel.z, correct_bno_data.accel.z, erroneous_bno);
	sensor_elimination(bno_data[0].gyro.x, bno_data[1].gyro.x, bno_data[2].gyro.x, correct_bno_data.gyro.x, erroneous_bno);
	sensor_elimination(bno_data[0].gyro.y, bno_data[1].gyro.y, bno_data[2].gyro.y, correct_bno_data.gyro.y, erroneous_bno);
	sensor_elimination(bno_data[0].gyro.z, bno_data[1].gyro.z, bno_data[2].gyro.z, correct_bno_data.gyro.z, erroneous_bno);
}

void bme_redundancy(uint8_t rslt_bme[MAX_SENSOR_NUMBER]) {
	// Redundancy of bno
	bool erroneous_bme[MAX_SENSOR_NUMBER] = {false};
	for (uint8_t i = 0; i<3; i++) {
			erroneous_bme[i] = rslt_bme[i];
	}
	sensor_elimination(bme_data[0].pressure, bme_data[1].pressure, bme_data[2].pressure, correct_bme_data.pressure, erroneous_bme);
	sensor_elimination(bme_data[0].temperature, bme_data[1].temperature, bme_data[2].temperature, correct_bme_data.temperature,erroneous_bme);
	sensor_elimination(bme_basepressure[0], bme_basepressure[1], bme_basepressure[2], correct_bme_basepressure, erroneous_bme);
}


