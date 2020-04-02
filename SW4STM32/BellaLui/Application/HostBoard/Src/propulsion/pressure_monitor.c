/*
 * pressure_monitor.c
 *
 *  Created on: Apr 2, 2020
 *      Author: Robin
 */

#include "stm32f4xx_hal.h"
#include <cmsis_os.h>

#include <propulsion/pressure_monitor.h>
#include <misc/Common.h>
#include <CAN_communication.h>
#include <CAN_handling.h>
#include <debug/led.h>

#define PRESSURE_MONITOR_PERIOD_MS (50)

#define DATA_ID_PP_MOTOR_PRESSURE 55 // TODO move to CAN-handling.h

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
		  can_setFrame(data, DATA_ID_PP_MOTOR_PRESSURE,timestamp);
		  osDelay(PRESSURE_MONITOR_PERIOD_MS);

	  }


}
float read_pressure_data ()
{
	// TODO Analog read data from port D0
	float data = 0;

	//HAL_ADC_

	return data;
}



