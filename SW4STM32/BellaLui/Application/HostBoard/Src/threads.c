/*
 * threads.c
 *
 *  Created on: 11 Feb 2020
 *      Author: Arion
 */

#include <can_reception.h>
#include <stddef.h>
#include <usart.h>
#include <threads.h>

#include <sync.h>
#include <debug/led.h>
#include <debug/shell.h>
#include <storage/flash_logging.h>
#include <storage/heavy_io.h>
#include <debug/console.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "Sensors/DataStructures.h"


osThreadId loggingHandle;
osThreadId task_ShellHandle;
osThreadId sdWriteHandle;
osThreadId task_ABHandle;
osThreadId sensorBoardHandle;
osThreadId task_LEDHandle;
osThreadId task_GPSHandle;
osThreadId telemetryTransmissionHandle;
osThreadId telemetryReceptionHandle;
osThreadId canReaderHandle;
osThreadId heavyIOHandle;
osThreadId kalmanHandle;
osThreadId rocketfsmHandle;
osThreadId state_estimatorHandle;
osThreadId presureMonitorHandle;


void create_semaphores() {
	init_heavy_scheduler();
	init_logging();
}


int32_t seed;
float rand() {
	seed += 0xC0FFEE;
	seed *= 666;
	return seed / (1 << 31);
}

void test_thread() {

	struct IMUData imuData = { 0 };
	struct AltitudeData altitudeData = { 0 };

	while(1) {
		sync_logic(10);
		uint32_t time = HAL_GetTick();

		imuData.accel.x = rand();
		imuData.accel.y = rand();
		imuData.accel.z = rand();
		imuData.gyro.x = rand();
		imuData.gyro.y = rand();
		imuData.gyro.z = rand();
		altitudeData.temperature = rand();
		altitudeData.pressure = rand();
		altitudeData.altitude = rand();

		can_setFrame((int32_t) imuData.accel.x, DATA_ID_ACCELERATION_X, time);
		can_setFrame((int32_t) imuData.accel.y, DATA_ID_ACCELERATION_Y, time);
		can_setFrame((int32_t) imuData.accel.z, DATA_ID_ACCELERATION_Z, time);
		can_setFrame((int32_t) (1000 * imuData.gyro.x), DATA_ID_GYRO_X, time);
		can_setFrame((int32_t) (1000 * imuData.gyro.y), DATA_ID_GYRO_Y, time);
		can_setFrame((int32_t) (1000 * imuData.gyro.z), DATA_ID_GYRO_Z, time);
		can_setFrame((int32_t) altitudeData.temperature, DATA_ID_TEMPERATURE, time);
		can_setFrame((int32_t) (altitudeData.pressure * 100), DATA_ID_PRESSURE, time);
		can_setFrame((int32_t) (altitudeData.altitude), DATA_ID_ALTITUDE, time);
	}
}

void create_threads() {
	osThreadDef(task_LED, TK_led_handler, osPriorityNormal, 0, 512);
	task_LEDHandle = osThreadCreate(osThread(task_LED), NULL);
	rocket_boot_log("LED thread started.\r\n");

	osThreadDef(task_shell, TK_shell, osPriorityNormal, 0, 512);
	task_ShellHandle = osThreadCreate(osThread(task_shell), NULL);
	rocket_boot_log("Shell thread started.\r\n");

	osThreadDef(0can_reader, TK_can_reader, osPriorityNormal, 0, 1024);
	canReaderHandle = osThreadCreate(osThread(0can_reader), NULL);
	rocket_boot_log("CAN reception thread started.\r\n");

	osThreadDef(1heavy_io, TK_heavy_io_scheduler, osPriorityNormal, 0, 1024);
	heavyIOHandle = osThreadCreate(osThread(1heavy_io), NULL);
	rocket_boot_log("Heavy IO thread started.\r\n");

	#ifdef FLASH_LOGGING
	 osThreadDef(2task_logging, TK_logging_thread, osPriorityNormal, 0, 512);
	 loggingHandle = osThreadCreate(osThread(2task_logging), NULL);
	 rocket_boot_log("Logging thread started.\r\n");
	#endif

	#ifdef AB_CONTROL
	  ab_init(&huart1);

	  osThreadDef(3task_AB, TK_ab_controller, osPriorityNormal, 0, 256);
	  task_ABHandle = osThreadCreate(osThread(3task_AB), NULL);
	  rocket_boot_log("Airbrakes thread started.\r\n");

	  osThreadDef(4state_estimator, TK_state_estimation, osPriorityNormal, 0, 256);
	  state_estimatorHandle = osThreadCreate(osThread(4state_estimator), NULL);
	  rocket_boot_log("State estimator thread started.\r\n");
	#endif

	/*#ifdef SDCARD
	  osThreadDef(sdWrite, TK_sd_sync, osPriorityNormal, 0, 1024);
	  sdWriteHandle = osThreadCreate(osThread(sdWrite), NULL);
	  rockerocket_boot_logt_log("SD card thread started.\r\n");
	#endif*/

	#ifdef SENSOR
	  osThreadDef(3sensor_board, TK_sensor_acquisition, osPriorityNormal, 0, 1024);
	  sensorBoardHandle = osThreadCreate(osThread(3sensor_board), NULL);
	  rocket_boot_log("Sensor acquisition thread started.\r\n");
	#endif

	#ifdef XBEE
	  xbee_freertos_init(&huart1);
	  osThreadDef(3xBeeTransmission, TK_xBeeTransmit, osPriorityNormal, 0, 2048);
	  telemetryTransmissionHandle = osThreadCreate(osThread(3xBeeTransmission), NULL);
	  rocket_boot_log("Telemetry transmission thread started.\r\n");
	  osThreadDef(4xBeeReception, TK_xBeeReceive, osPriorityNormal, 0, 2048);
	  telemetryReceptionHandle = osThreadCreate(osThread(4xBeeReception), NULL);
	  rocket_boot_log("Telemetry reception thread started.\r\n");
	#endif

	/*#ifdef KALMAN
	  osThreadDef(5kalman, TK_kalman, osPriorityNormal, 0, 2048); // Kalman needs big stack
	  kalmanHandle = osThreadCreate(osThread(5kalman), NULL);
	  rocket_boot_log("Kalman thread started.\n");
	#endif*/

	#ifdef ROCKET_FSM
	  osThreadDef(6rocket_fsm, TK_state_machine, osPriorityNormal, 0, 512);
	  rocketfsmHandle = osThreadCreate(osThread(6rocket_fsm), NULL);
	  rocket_boot_log("FSM thread started.\r\n");
	#endif

	#ifdef PRESSURE_MONITORING
	  osThreadDef(7pressure_monitor, TK_pressure_monitor, osPriorityNormal, 0, 256);
	  presureMonitorHandle = osThreadCreate(osThread(7pressure_monitor), NULL);
	  rocket_boot_log("Pressure monitoring thread started.\r\n");
	#endif

	#ifdef FLASH_DUMP_BOARD
	  on_fullsd_dump_request();
	#endif


	#ifdef DEBUG_TASK
	  osThreadDef(8test_thread, test_thread, osPriorityNormal, 0, 512);
	  rocketfsmHandle = osThreadCreate(osThread(8test_thread), NULL);
	  rocket_boot_log("Test thread started.\r\n");
	#endif
}
