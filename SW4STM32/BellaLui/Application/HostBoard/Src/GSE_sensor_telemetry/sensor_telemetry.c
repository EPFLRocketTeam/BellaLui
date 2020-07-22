/*
 * sensor_telemetry.c
 *
 *  Created on: 8 Jul 2020
 *      Author: lucaspallez
 */


#include <cmsis_os.h>
#include <CAN_communication.h>
#include <debug/console.h>
#include <GSE_sensor_telemetry/sensor_telemetry.h>
#include <telemetry/telemetry_handling.h>
#include <telemetry/xbee.h>
#include <stm32f4xx_hal.h>
#include <sys/_stdint.h>

void telemetry_init(void)
{
	//Initialise telemetry

}
void sensors_init(void)
{
	//Initialize sensors

}

void TK_telemetry_control(void const * argument)
{
	//Order status, check each state and send appropriate data
	for(;;)
	{
		osDelay(1000);
	}
}
void TK_sensors_control(void const * argument)
{
	for(;;)
	{
		osDelay(1000);
	}
}
