/*
 * pressure_monitor.c
 *
 *  Created on: Apr 2, 2020
 *      Author: Robin
 */

#include <can_reception.h>
#include <can_transmission.h>
#include "stm32f4xx_hal.h"
#include <cmsis_os.h>

#include <propulsion/pressure_monitor.h>
#include <misc/Common.h>
#include <debug/led.h>

#define PRESSURE_MONITOR_PERIOD_MS (50)


//Should be launched as a thread in thread.c only on the propulsion board
void TK_pressure_monitor (void const * argument)
{

	  float pressure_data = 0;
	  uint32_t data;
	  uint32_t timestamp;
	  for (;;) {

		  //obtain pressure data from sensor
		  pressure_data = read_pressure_data();

		  data = *((uint32_t*)(&pressure_data));
		  timestamp = HAL_GetTick();
		  // send data over can bus
		  can_setFrame(data, DATA_ID_MOTOR_PRESSURE,timestamp);
		  osDelay(PRESSURE_MONITOR_PERIOD_MS);

	  }


}
float read_pressure_data ()
{
	// TODO Check if right ADC is chosen, and configuration is right
	float data = 0;
	float obtained_value = 0;

	//obtained_value =  HAL_ADC_getValue()
	 LL_ADC_REG_StartConversion(ADC1);
	 while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0);
	 if( LL_ADC_IsActiveFlag_EOC(ADC1) ) LL_ADC_ClearFlag_EOC(ADC1);
	 obtained_value = LL_ADC_REG_ReadConversionData12(ADC1);

	data = ((float)obtained_value/4095.0)*3.3;

	return data;
}



