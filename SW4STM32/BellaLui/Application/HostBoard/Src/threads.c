/*
 * threads.c
 *
 *  Created on: 11 Feb 2020
 *      Author: Arion
 */

#include <stddef.h>
#include <usart.h>
#include <threads.h>

#include <CAN_handling.h>
#include <sync.h>
#include <debug/led.h>
#include <storage/flash_logging.h>
#include <storage/heavy_io.h>
#include <debug/console.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"



osThreadId loggingHandle;
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
osThreadId GSEValveHandle;
osThreadId GSEIgnitionHandle;
osThreadId GSECodeHandle;
osThreadId GSESensorHandle;
osThreadId GSETelemetryHandle;

void create_semaphores() {
	init_heavy_scheduler();
	init_logging();
}

void create_threads() {
	osThreadDef(task_LED, TK_led_handler, osPriorityNormal, 0, 256);
	task_LEDHandle = osThreadCreate(osThread(task_LED), NULL);
	rocket_log("LED thread started.\n");

	osThreadDef(can_reader, TK_can_reader, osPriorityNormal, 0, 1024);
	canReaderHandle = osThreadCreate(osThread(can_reader), NULL);
	rocket_log("CAN reception thread started.\n");

	osThreadDef(heavy_io, TK_heavy_io_scheduler, osPriorityNormal, 0, 1024);
	heavyIOHandle = osThreadCreate(osThread(heavy_io), NULL);
	rocket_log("Heavy IO thread started.\n");

	#ifdef GPS
	  osThreadDef(task_GPSHandle, TK_GPS_board, osPriorityNormal, 0, 256);
	  task_GPSHandle = osThreadCreate(osThread(task_GPSHandle), NULL);
	  rocket_log("GPS thread started.\n");
	  gps_init(&huart6);
	#endif

	#ifdef AB_CONTROL
	  ab_init(&huart1);

	  osThreadDef(task_AB, TK_ab_controller, osPriorityNormal, 0, 256);
	  task_ABHandle = osThreadCreate(osThread(task_AB), NULL);
	  rocket_log("Airbrakes thread started.\n");

	  osThreadDef(state_estimator, TK_state_estimation, osPriorityNormal, 0, 256);
	  state_estimatorHandle = osThreadCreate(osThread(state_estimator), NULL);
	  rocket_log("State estimator thread started.\n");
	#endif

	#ifdef FLASH_LOGGING
	 osThreadDef(task_logging, TK_logging_thread, osPriorityNormal, 0, 256);
	 loggingHandle = osThreadCreate(osThread(task_logging), NULL);
	 rocket_log("Logging thread started.\n");
	#endif

	#ifdef SDCARD
	  osThreadDef(sdWrite, TK_sd_sync, osPriorityNormal, 0, 1024);
	  sdWriteHandle = osThreadCreate(osThread(sdWrite), NULL);
	  rocket_log("SD card thread started.\n");
	#endif

	#ifdef SENSOR
	  osThreadDef(sensor_board, TK_sensor_board, osPriorityNormal, 0, 1024);
	  sensorBoardHandle = osThreadCreate(osThread(sensor_board), NULL);
	  rocket_log("Sensor acquisition thread started.\n");
	#endif

	#ifdef XBEE
	  xbee_freertos_init(&huart1);
	  osThreadDef(xBeeTransmission, TK_xBeeTransmit, osPriorityNormal, 0, 128);
	  telemetryTransmissionHandle = osThreadCreate(osThread(xBeeTransmission), NULL);
	  rocket_log("Telemetry transmission thread started.\n");
	  osThreadDef(xBeeReception, TK_xBeeReceive, osPriorityNormal, 0, 128);
	  telemetryReceptionHandle = osThreadCreate(osThread(xBeeReception), NULL);
	  rocket_log("Telemetry reception thread started.\n");
	#endif

	#ifdef KALMAN
	  osThreadDef(kalman, TK_kalman, osPriorityNormal, 0, 1024);
	  kalmanHandle = osThreadCreate(osThread(kalman), NULL);
	  rocket_log("Kalman thread started.\n");
	#endif

	#ifdef ROCKET_FSM
	  osThreadDef(rocket_fsm, TK_state_machine, osPriorityNormal, 0, 256);
	  rocketfsmHandle = osThreadCreate(osThread(rocket_fsm), NULL);
	  rocket_log("FSM thread started.\n");
	#endif

	#ifdef PRESSURE_MONITORING
	  osThreadDef(pressure_monitor, TK_pressure_monitor, osPriorityNormal, 0, 256);
	  presureMonitorHandle = osThreadCreate(osThread(pressure_monitor), NULL);
	  rocket_log("Pressure monitoring thread started.\n");
	#endif

	#ifdef VALVE
	  valve_init();
	  osThreadDef(GSE_valves, TK_GSE_valve_control, osPriorityNormal, 0, 128);
	  GSEValveHandle = osThreadCreate(osThread(GSE_valves), NULL);
	  rocket_log("Valve control thread started. \n");
	#endif

	#ifdef VALVE
	  valve_init();
	  osThreadDef(GSE_valves, TK_GSE_valve_control, osPriorityNormal, 0, 128);
	  GSEValveHandle = osThreadCreate(osThread(GSE_valves), NULL);
	  rocket_log("Valve control thread started. \n");
	#endif

	#ifdef IGNITION
	  ignition_sys_init();
	  osThreadDef(ignition, TK_ignition_control, osPriorityNormal, 0, 128);
		  GSEIgnitionHandle = osThreadCreate(osThread(ignition), NULL);
	  rocket_log("Ignition control thread started.\n");
	#endif

	#ifdef SECURITY_CODE
	  code_init();
	  osThreadDef(security_code, TK_code_control, osPriorityNormal, 0, 128);
		  GSECodeHandle = osThreadCreate(osThread(security_code), NULL);
	  rocket_log("Security Code control thread started.\n");
	#endif


#ifdef SENSOR_TELEMETRY
//	  	telemetry_init();
//	  	sensors_init();
  	osThreadDef(GSE_telemetry, TK_telemetry_control, osPriorityNormal, 0, 128);
  		  	  GSETelemetryHandle = osThreadCreate(osThread(GSE_telemetry), NULL);
  	rocket_log("GSE Telemetry thread started.\n");
//		osThreadDef(GSE_sensor, TK_sensors_control, osPriorityNormal, 0, 128);
//				  GSESensorHandle = osThreadCreate(osThread(GSE_sensor), NULL);
//		rocket_log("GSE Sensors thread started.\n");
#endif
}
