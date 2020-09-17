/*
 * sensor_telemetry.c
 *
 *  Created on: 8 Jul 2020
 *      Author: lucaspallez
 */

#include "stm32f4xx_hal.h"
#include <stm32f4xx_hal_conf.h>

#include <cmsis_os.h>
#include <CAN_communication.h>
#include <GSE_sensor_telemetry/sensor_telemetry.h>
#include <main.h>
#include <stm32_hal_legacy.h>
#include <stm32f446xx.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_hal_adc.h>

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
//		float tank_temp = 0;
//		float hose_pressure = 0;
//		float host_temp = 0;
//
//		//Tank Temperature
//		tank_temp = read_tank_temp();
//		can_setFrame(tank_temp, DATA_ID_TANK_TEMPERATURE, HAL_GetTick());
//
//
		osDelay(1000);
	}
}

float read_tank_temp()
		{
			float temp = 0;


			return temp;
		}
