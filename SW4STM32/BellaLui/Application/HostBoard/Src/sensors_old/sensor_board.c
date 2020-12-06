/**
  ******************************************************************************
  * File Name          : sensor_board.c
  * Description        : This file provides code for the configuration
  *                      of the sensor board.
  * Author 			   : Alexandre Devienne, Quentin Delfosse, Arion Zimmermann
  * Date    		   : Feb 2020

  * Notes : The code below was developed to implement a redundancy on the 4 sensors
  * 		used by this board, but can still work with the previous versions of the
  * 		sensor boards.

  ******************************************************************************
  */


#include "debug/profiler.h"
#include "debug/led.h"
#include "debug/console.h"
#include "debug/monitor.h"
#include "can_transmission.h"
#include "sync.h"

#include <stm32f4xx_hal.h>
#include <cmsis_os.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>

#include <sensors_old/BME280/bme280.h>
#include <sensors_old/BNO055/bno055.h>
#include <sensors_old/redundancy.h>
#include <sensors_old/sensor_board.h>
#include <sensors_old/sensors.h>


#define I2C_TIMEOUT 3
#define FMPI2C_TIMEOUT 3
#define BARO_CALIB_N 128

#define TICKS_BETWEEN_INIT_ATTEMPTS 64 // At 10 Hz, corresponds to ca. 5 seconds

#define ACCEL_ACQUISITION_DIVIDER 1
#define GYRO_ACQUISITION_DIVIDER 10

/* sensor_id is the index of the sensor, which is different form the dev_id ( defined in bme280_dev)
 * or dev_addr ( defined in bno055_t) representing its address for the I2C protocol
 */


struct bno055_data {
	struct bno055_accel_float_t accel;
	struct bno055_gyro_float_t gyro;
	struct bno055_mag_float_t mag;
};

struct bme280_data_float {
	float temperature;
	float pressure;
	float base_pressure;
};


static int8_t init_barometer(uint8_t sensor_id);
static int8_t init_accelerometer(uint8_t sensor_id);
static int8_t fetch_barometer(uint8_t sensor_id, struct bme280_data_float* data);
static int8_t fetch_accelerometer(uint8_t sensor_id, struct bno055_data* data);


static int8_t bno_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* data, uint8_t len);
static int8_t bno_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* data, uint8_t len);
static int8_t bme_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t* data, uint16_t len);
static int8_t bme_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t* data, uint16_t len);
static void delay_ms(uint32_t delay);

static struct bme280_data_float barometer_redundancy(struct bme280_data_float* data, uint8_t count);
static struct bno055_data accelerometer_redundancy(struct bno055_data* data, uint8_t count);


extern I2C_HandleTypeDef hi2c3;
extern FMPI2C_HandleTypeDef hfmpi2c1;

uint8_t current_sensor = 0;

//## BME280 ## Barometer
struct bme280_dev barometers[MAX_BAROMETERS];
struct bme280_data_float barometer_data;
bool available_barometers[MAX_BAROMETERS];

//## BNO055 ## IMU
struct bno055_t accelerometers[MAX_ACCELEROMETERS];
struct bno055_data accelerometer_data;
bool available_accelerometers[MAX_ACCELEROMETERS];

/*
 * Main function used to initialize and fetch the sensors.
 *
 * If at least one BME or BNO has been initialized, the redundancy
 * algorithm is called.
 */


void TK_sensor_board(void const * argument) {
	uint16_t retry_counter = 1;

	uint8_t led_barometer = led_register_TK();
	uint8_t led_accelerometer = led_register_TK();

	struct bme280_data_float full_barometer_data[MAX_BAROMETERS];
	struct bno055_data full_accelerometer_data[MAX_ACCELEROMETERS];

	while(true) {
		uint8_t num_barometer_data = 0;
		uint8_t num_accelerometer_data = 0;

		start_profiler(1); // Whole thread profiling

		start_profiler(1); // Initialisation of sensors


		if(available_barometers[0] == 0 && available_barometers[1] == 0 && retry_counter % TICKS_BETWEEN_INIT_ATTEMPTS == 0) {
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

		if(available_barometers[2] == 0 && available_barometers[3] == 0 && retry_counter % TICKS_BETWEEN_INIT_ATTEMPTS == 0) {
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

		for(uint8_t i = 0; i < MAX_BAROMETERS; i++) {
			if(!available_barometers[i] && retry_counter % TICKS_BETWEEN_INIT_ATTEMPTS == 0) {
				int8_t status = init_barometer(i);

				if(status == BME280_OK) {
					available_barometers[i] = true;
					rocket_log("Barometer %u initialised\n", i);
				}
			}
		}

		for(uint8_t i = 0; i < MAX_ACCELEROMETERS; i++) {
			if(!available_accelerometers[i] && retry_counter % TICKS_BETWEEN_INIT_ATTEMPTS == 0) {
				int32_t status = init_accelerometer(i);

				if(status == BNO055_SUCCESS) {
					available_accelerometers[i] = true;
					rocket_log("Accelerometer %u initialised\n", i);
				}
			}
		}

		retry_counter++;

		end_profiler();


		start_profiler(2); // BNO fetching (task ID higher than 20)

		for(uint8_t i = 0; i < MAX_BAROMETERS; i++) {
			if(available_barometers[i]) {
				if(fetch_barometer(i, &full_barometer_data[num_barometer_data]) == BME280_OK) {
					num_barometer_data++;
				} else {
					available_barometers[i] = false;
					rocket_log("Barometer %u ceased functioning\n", i);
				}
			}
		}

		end_profiler();


		start_profiler(3); // BME fetching (task ID higher than 40)

		for(uint8_t i = 0; i < MAX_ACCELEROMETERS; i++) {
			if(available_accelerometers[i]) {
				if(fetch_accelerometer(i, &full_accelerometer_data[num_accelerometer_data]) == BNO055_SUCCESS) {
					num_accelerometer_data++;
				} else {
					available_accelerometers[i] = false;
					rocket_log("Accelerometer %u ceased functioning\n", i);
				}
			}
		}

		end_profiler();


		start_profiler(4); // Redundancy and data filtering (task ID higher than 60)

		if(num_barometer_data > 0) {
			barometer_data = barometer_redundancy(full_barometer_data, num_barometer_data);
			led_set_TK_rgb(led_barometer, 0x00, 0xFF, 0x00);
		} else {
			led_set_TK_rgb(led_barometer, 0xFF, 0x00, 0x00);
		}

		if(num_accelerometer_data > 0) {
			accelerometer_data = accelerometer_redundancy(full_accelerometer_data, num_accelerometer_data);
			led_set_TK_rgb(led_accelerometer, 0x00, 0xFF, 0x00);
		} else {
			led_set_TK_rgb(led_accelerometer, 0xFF, 0x00, 0x00);
		}

		end_profiler();

		start_profiler(5); // Transmit data through the CAN bus (task ID higher than 80)

		uint32_t time = HAL_GetTick();

		can_setFrame((int32_t) barometer_data.temperature, DATA_ID_TEMPERATURE, time);
		can_setFrame((int32_t) (barometer_data.pressure * 100), DATA_ID_PRESSURE, time);
		can_setFrame((int32_t) accelerometer_data.accel.x, DATA_ID_ACCELERATION_X, time);
		can_setFrame((int32_t) accelerometer_data.accel.y, DATA_ID_ACCELERATION_Y, time);
		can_setFrame((int32_t) accelerometer_data.accel.z, DATA_ID_ACCELERATION_Z, time);
		can_setFrame((int32_t) (1000 * accelerometer_data.gyro.x), DATA_ID_GYRO_X, time);
		can_setFrame((int32_t) (1000 * accelerometer_data.gyro.y), DATA_ID_GYRO_Y, time);
		can_setFrame((int32_t) (1000 * accelerometer_data.gyro.z), DATA_ID_GYRO_Z, time);

		end_profiler();

		end_profiler();


		if(enter_monitor(SENSOR_MONITOR)) {
			rocket_log(" Available barometers: %d\x1b[K\n", num_barometer_data);
			rocket_log(" Available accelerometers: %d\x1b[K\n", num_accelerometer_data);
			rocket_log(" Temperature: %d [m°C]\x1b[K\n", (uint32_t) (barometer_data.temperature * 10));
			rocket_log(" Pressure: %d [Pa]\x1b[K\n", (uint32_t) (barometer_data.pressure));
			rocket_log(" Acceleration X: %d [mg]\x1b[K\n", (uint32_t) (accelerometer_data.accel.x));
			rocket_log(" Acceleration Y: %d [mg]\x1b[K\n", (uint32_t) (accelerometer_data.accel.y));
			rocket_log(" Acceleration Z: %d [mg]\x1b[K\n", (uint32_t) (accelerometer_data.accel.z));
			rocket_log(" Rotation X: %d [mrad/s]\x1b[K\n", (uint32_t) (1000 * accelerometer_data.gyro.x));
			rocket_log(" Rotation Y: %d [mrad/s]\x1b[K\n", (uint32_t) (1000 * accelerometer_data.gyro.y));
			rocket_log(" Rotation Z: %d [mrad/s]\x1b[K\n\x1b[K\n", (uint32_t) (1000 * accelerometer_data.gyro.z));

			exit_monitor(SENSOR_MONITOR);
		}

	    sync_logic(10);
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


int8_t init_barometer(uint8_t sensor_id) {
	current_sensor = sensor_id;

	if(sensor_id == 1 || sensor_id == 2) {
		barometers[sensor_id].dev_id = BME280_I2C_ADDR_PRIM;
	} else if(sensor_id == 0 || sensor_id == 3) {
		barometers[sensor_id].dev_id = BME280_I2C_ADDR_SEC;
	}

	barometers[sensor_id].intf = BME280_I2C_INTF;
	barometers[sensor_id].read = &bme_i2c_read;
	barometers[sensor_id].write = &bme_i2c_write;
	barometers[sensor_id].delay_ms = &delay_ms;

	int8_t result = bme280_init(&barometers[sensor_id]); // Returns 1 if error

	if(result != BME280_OK) {
		// rocket_log("Barometer %u failed to initialise with error code %d\n", sensor_id, result);
		return result;
	}
	//Always read the current settings before writing
	result = bme280_get_sensor_settings(&barometers[sensor_id]);

	if(result != BME280_OK) {
		// rocket_log("Barometer %u failed to fetch sensor settings with error code %d\n", sensor_id, result);
		return result;
	}
	//Overwrite the desired settings
	barometers[sensor_id].settings.filter = BME280_FILTER_COEFF_OFF;
	barometers[sensor_id].settings.osr_p = BME280_OVERSAMPLING_8X;
	barometers[sensor_id].settings.osr_t = BME280_OVERSAMPLING_1X;
	barometers[sensor_id].settings.standby_time = BME280_STANDBY_TIME_0_5_MS;


	result = bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &barometers[sensor_id]);

	if(result != BME280_OK) {
		// rocket_log("Barometer %u failed to change sensor settings with error code %d\n", sensor_id, result);
		return result;
	}

	//Always set the power mode after setting the configuration
	result = bme280_set_sensor_mode(BME280_NORMAL_MODE, &barometers[sensor_id]);

	if(result != BME280_OK) {
		// rocket_log("Barometer %u failed to change power mode with error code %d\n", sensor_id, result);
		return result;
	}

	return BME280_OK;
}

int8_t init_accelerometer(uint8_t sensor_id) {
	current_sensor = sensor_id;

	if(sensor_id == 1 || sensor_id == 2) {
		accelerometers[sensor_id].dev_addr = BNO055_I2C_ADDR1;
	} else if(sensor_id == 0 || sensor_id == 3) {
		accelerometers[sensor_id].dev_addr = BNO055_I2C_ADDR2;
	}

	accelerometers[sensor_id].bus_write = &bno_i2c_write;
	accelerometers[sensor_id].bus_read = &bno_i2c_read;
	accelerometers[sensor_id].delay_msec = (void(*)(u32)) &delay_ms;

	int8_t result = bno055_init(&accelerometers[sensor_id]); // Returns 1 if error

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %u failed to initialise with error code %d\n", sensor_id, result);
		return result;
	}

	result = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %u failed to set power mode with error code %d\n", sensor_id, result);
		return result;
	}

	result = bno055_set_operation_mode(BNO055_OPERATION_MODE_ACCGYRO);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %u failed to set operation mode with error code %d\n", sensor_id, result);
		return result;
	}

	result = bno055_set_accel_range(BNO055_ACCEL_RANGE_16G);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %u failed to set acceleration range with error code %d\n", sensor_id, result);
		return result;
	}

	result = bno055_set_accel_bw(BNO055_ACCEL_BW_250HZ);
	// result = bno055_set_accel_bw(BNO055_ACCEL_BW_500HZ);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %u failed to set acceleration bandwidth with error code %d\n", sensor_id, result);
		return result;
	}

	result = bno055_set_accel_unit(BNO055_ACCEL_UNIT_MG);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %u failed to set acceleration unit with error code %d\n", sensor_id, result);
		return result;
	}

	result = bno055_set_gyro_bw(BNO055_GYRO_BW_230HZ);
	// result = bno055_set_gyro_bw(BNO055_GYRO_BW_523HZ);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %u failed to set gyroscope bandwidth with error code %d\n", sensor_id, result);
		return result;
	}

	result = bno055_set_gyro_unit(BNO055_GYRO_UNIT_RPS);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %u failed to set gyroscope unit with error code %d\n", sensor_id, result);
		return result;
	}

	result = bno055_write_page_id(BNO055_PAGE_ZERO);

	if(result != BNO055_SUCCESS) {
		// rocket_log("Accelerometer %u failed to set default write page ID with error code %d\n", sensor_id, result);
		return result;
	}

	return BNO055_SUCCESS;
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

int8_t fetch_barometer(uint8_t sensor_id, struct bme280_data_float* data) {
	static struct bme280_data raw_data;

	current_sensor = sensor_id;

	int8_t result = bme280_get_sensor_data(BME280_TEMP | BME280_PRESS, &raw_data, &barometers[sensor_id]);

	data->temperature = (float) raw_data.temperature;
	data->pressure = (float) raw_data.pressure / 100;

	return result;
}

int8_t fetch_accelerometer(uint8_t sensor_id, struct bno055_data* data) {
	static uint8_t accel_data[BNO055_ACCEL_XYZ_DATA_SIZE];
	static uint8_t gyro_data[BNO055_GYRO_XYZ_DATA_SIZE];
	static uint8_t accel_counter = 0;
	static uint8_t gyro_counter = 0;

	current_sensor = sensor_id;

	uint8_t result = 0;

	if(accel_counter++ == ACCEL_ACQUISITION_DIVIDER) {
		result += bno_i2c_read(accelerometers[sensor_id].dev_addr, BNO055_ACCEL_DATA_X_LSB_VALUEX_REG, accel_data, BNO055_ACCEL_XYZ_DATA_SIZE);
		accel_counter = 0;
	}

	if(gyro_counter++ == GYRO_ACQUISITION_DIVIDER) {
		result += bno_i2c_read(accelerometers[sensor_id].dev_addr, BNO055_GYRO_DATA_X_LSB_VALUEX_REG, gyro_data, BNO055_GYRO_XYZ_DATA_SIZE);
		gyro_counter = 0;
	}

	data->accel.x = (float) ((((int16_t) ((int8_t) accel_data[1]) << 8) | accel_data[0]) / BNO055_ACCEL_DIV_MG);
	data->accel.y = (float) ((((int16_t) ((int8_t) accel_data[3]) << 8) | accel_data[2]) / BNO055_ACCEL_DIV_MG);
	data->accel.z = (float) ((((int16_t) ((int8_t) accel_data[5]) << 8) | accel_data[4]) / BNO055_ACCEL_DIV_MG);
	data->gyro.x = (float) ((((int16_t) ((int8_t) gyro_data[1]) << 8) | gyro_data[0]) / BNO055_GYRO_DIV_RPS);
	data->gyro.y = (float) ((((int16_t) ((int8_t) gyro_data[3]) << 8) | gyro_data[2]) / BNO055_GYRO_DIV_RPS);
	data->gyro.z = (float) ((((int16_t) ((int8_t) gyro_data[5]) << 8) | gyro_data[4]) / BNO055_GYRO_DIV_RPS);

	return result;
}



/*
 * Redundancy functions
 *
 * Calls the redundancy algorithm for each kind of data given by the sensors
 *
 */

struct bme280_data_float barometer_redundancy(struct bme280_data_float* data, uint8_t count) {
	struct bme280_data_float output;

	float temperature[count];
	float pressure[count];
	float base_pressure[count];

	for(uint8_t i = 0; i < count; i++) {
		temperature[i] = data[i].temperature;
		pressure[i] = data[i].pressure;
		base_pressure[i] = data[i].base_pressure;
	}

	output.temperature = get_filtered_sensor_output(temperature, count);
	output.pressure = get_filtered_sensor_output(pressure, count);
	output.base_pressure = get_filtered_sensor_output(base_pressure, count);

	return output;
}

struct bno055_data accelerometer_redundancy(struct bno055_data* data, uint8_t count) {
	struct bno055_data output;

	float accelX[count];
	float accelY[count];
	float accelZ[count];
	float gyroX[count];
	float gyroY[count];
	float gyroZ[count];

	for(uint8_t i = 0; i < count; i++) {
		accelX[i] = data[i].accel.x;
		accelY[i] = data[i].accel.y;
		accelZ[i] = data[i].accel.z;
		gyroX[i] = data[i].gyro.x;
		gyroY[i] = data[i].gyro.y;
		gyroZ[i] = data[i].gyro.z;
	}

	output.accel.x = get_filtered_sensor_output(accelX, count);
	output.accel.y = get_filtered_sensor_output(accelY, count);
	output.accel.z = get_filtered_sensor_output(accelZ, count);
	output.gyro.x = get_filtered_sensor_output(gyroX, count);
	output.gyro.y = get_filtered_sensor_output(gyroY, count);
	output.gyro.z = get_filtered_sensor_output(gyroZ, count);

	return output;
}

float mean(float* data, uint8_t count) {
	float sum = 0.0f;

	for(uint8_t i = 0; i < count; i++) {
		sum += data[i];
	}

	return sum / count;
}

/*
 * Low level i2c functions :
 *
 * I2C3 : Sensor 0 and Sensor 1
 * FMPI2C1 : Sensor 2 and Sensor 3
 *
 */

int8_t bno_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len) { // The function signature must have a 8-bit length
	return bme_i2c_read(dev_id, reg_addr, data, len);
}

int8_t bno_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint8_t len) { // The function signature must have a 8-bit length
	return bme_i2c_write(dev_id, reg_addr, data, len);
}

int8_t bme_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t rslt = 0;

	vTaskSuspendAll();

	if (current_sensor == 0 || current_sensor == 1) {
		rslt = HAL_I2C_Mem_Read(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	} else if (current_sensor == 2 || current_sensor == 3) {
		rslt = HAL_FMPI2C_Mem_Read(&hfmpi2c1, dev_id << 1, reg_addr, FMPI2C_MEMADD_SIZE_8BIT, data, len, FMPI2C_TIMEOUT);
	}

	xTaskResumeAll();

	return rslt;
}

int8_t bme_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	int8_t rslt = 0;

	vTaskSuspendAll();

	if (current_sensor == 0 || current_sensor == 1) {
		rslt = HAL_I2C_Mem_Write(&hi2c3, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
	} else if (current_sensor == 2 || current_sensor == 3) {
		rslt = HAL_FMPI2C_Mem_Write(&hfmpi2c1, dev_id << 1, reg_addr, FMPI2C_MEMADD_SIZE_8BIT, data, len, FMPI2C_TIMEOUT);
	}

	xTaskResumeAll();

	return rslt;
}

void delay_ms(uint32_t delay) {
	osDelay(delay);
}
