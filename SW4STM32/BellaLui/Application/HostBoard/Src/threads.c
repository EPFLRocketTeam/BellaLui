/*
 * threads.c
 *
 *  Created on: 11 Feb 2020
 *      Author: Arion
 */

#define SENSOR
#define LED

#define FLASH_LOGGING
#define OS_STKCHECK



#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


#include <CAN_handling.h>
#include <debug/led.h>
#include <storage/flash_logging.h>
#include <storage/heavy_io.h>

#ifdef SENSOR
#include <sensors/sensor_board.h>
#endif


osThreadId loggingHandle;
osThreadId sdWriteHandle;
osThreadId task_ABHandle;
osThreadId sensorBoardHandle;
osThreadId task_LEDHandle;
osThreadId task_GPSHandle;
osThreadId xBeeTelemetryHandle;
osThreadId canReaderHandle;
osThreadId kalmanHandle;
osThreadId rocketfsmHandle;
osThreadId state_estimatorHandle;


void create_semaphores() {
	init_heavy_scheduler();
	init_logging();
}

void create_threads() {
	osThreadDef(task_LED, TK_led_handler, osPriorityNormal, 0, 256);
	task_LEDHandle = osThreadCreate(osThread(task_LED), NULL);

	osThreadDef(can_reader, TK_can_reader, osPriorityNormal, 0, 1024);
	canReaderHandle = osThreadCreate(osThread(can_reader), NULL);

	osThreadDef(heavy_io, TK_heavy_io_scheduler, osPriorityNormal, 0, 1024);
	canReaderHandle = osThreadCreate(osThread(heavy_io), NULL);


	#ifdef GPS
	  osThreadDef(task_GPSHandle, TK_GPS_board, osPriorityNormal, 0, 256);
	  task_GPSHandle = osThreadCreate(osThread(task_GPSHandle), NULL);
	  gps_init(&huart6);
	#endif

	#ifdef AB_CONTROL
	  osThreadDef(task_AB, TK_ab_controller, osPriorityNormal, 0, 256);
	  task_ABHandle = osThreadCreate(osThread(task_AB), NULL);
	  ab_init(&huart1);

	  osThreadDef(state_estimator, TK_state_estimation, osPriorityNormal, 0, 256);
	  state_estimatorHandle = osThreadCreate(osThread(state_estimator), NULL);
	#endif

	#ifdef FLASH_LOGGING
	 osThreadDef(task_logging, TK_logging_thread, osPriorityNormal, 0, 256);
	 loggingHandle = osThreadCreate(osThread(task_logging), NULL);
	#endif

	#ifdef SDCARD
	  osThreadDef(sdWrite, TK_sd_sync, osPriorityNormal, 0, 1024);
	  sdWriteHandle = osThreadCreate(osThread(sdWrite), NULL);
	#endif

	#ifdef SENSOR
	  osThreadDef(sensor_board, TK_sensor_board, osPriorityNormal, 0, 1024);
	  sensorBoardHandle = osThreadCreate(osThread(sensor_board), NULL);
	#endif

	#ifdef XBEE
	  xbee_freertos_init(&huart1);
	  osThreadDef(xBeeTelemetry, TK_xBeeTelemetry, osPriorityNormal, 0, 128);
	  xBeeTelemetryHandle = osThreadCreate(osThread(xBeeTelemetry), NULL);
	#endif

	#ifdef KALMAN
	  osThreadDef(kalman, TK_kalman, osPriorityNormal, 0, 1024);
	  kalmanHandle = osThreadCreate(osThread(kalman), NULL);
	#endif

	#ifdef ROCKET_FSM
	  osThreadDef(rocket_fsm, TK_state_machine, osPriorityNormal, 0, 256);
	  rocketfsmHandle = osThreadCreate(osThread(rocket_fsm), NULL);
	#endif
}
