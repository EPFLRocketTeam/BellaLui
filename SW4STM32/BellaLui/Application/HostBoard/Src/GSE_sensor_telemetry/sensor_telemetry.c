/*
 * sensor_telemetry.c
 *
 *  Created on: 8 Jul 2020
 *      Author: lucaspallez
 */
#include "main.h"
#include <cmsis_os.h>
#include <FreeRTOSConfig.h>
#include <GSE_sensor_telemetry/sensor_telemetry.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f4xx_hal_tim.h>
#include <sys/_stdint.h>
#include <task.h>
#include <tim.h>
#include "adc.h"
#include "dma.h"

static uint32_t GSE_sensorData[NB_SENSOR];

static uint32_t adcBuffer[NB_SENSOR];
static uint32_t time;

static SENSOR_DATA_t current_data;

static SAMPLING_DATA_t sampling = {0};

static uint16_t counter = 0;

void telemetry_init(void)
{
	//Initialise telemetry

}
void sensors_init(void)
{
	//Initialize sensors
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, NB_SENSOR);
	time = 0;
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	for(int i = 0; i < NB_SENSOR; i++) {
		GSE_sensorData[i] = adcBuffer[i];
	}
	sampling.tank_temp += adcBuffer[0];
	sampling.hose_pressure += adcBuffer[1];
	sampling.hose_temp += adcBuffer[2];
//	sampling.vbat += adcBuffer[3];


	counter++;
	if(counter == NB_SAMPLES) {
		time += SAMPLING_TIME;
		current_data.tank_temp = sampling.tank_temp>>6;
		current_data.hose_pressure = sampling.hose_pressure>>6;
		current_data.hose_temp = sampling.hose_temp>>6;
		current_data.vbat = sampling.vbat>>6;
		current_data.time = xTaskGetTickCountFromISR() * 1000 / configTICK_RATE_HZ;
		counter = 0;
		sampling.tank_temp = 0;
		sampling.hose_pressure = 0;
		sampling.hose_temp = 0;
//		sampling.vbat = 0;
//		static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		xSemaphoreGiveFromISR( get_can_sem(), &xHigherPriorityTaskWoken );
//		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	}
}
