/*
 * ignition.c
 *
 *  Created on: 8 Jul 2020
 *      Author: lucaspallez
 */

#include <cmsis_os.h>
#include <CAN_communication.h>
#include <CAN_handling.h>
#include <debug/console.h>
#include <GSE_code/code.h>
#include <GSE_ignition/ignition.h>
#include <misc/Common.h>
//#include <stm32_hal_legacy.h>
#include <stm32f446xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_ll_adc.h>
#include <sys/_stdint.h>


void ignition_sys_init(void)
{
	//Initialise GPIOs on S2 socket
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	rocket_log("Ignition systems initialised.\n");

	//Set Ignition values to low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

}


void TK_ignition_control(void const * argument)
{
	//if xbee code = ignition (D0)
	//if xbee code = ignition sec (D1)

	uint8_t disconnect_order = 0;
	uint8_t ignition_order = 0;
	float current = 0;

	//TODO Add sensor confirmation for main ignition and disconnect
	//TODO Add delay after ignition to shut it down automatically

	 for(;;)
	 {
		 if(verify_security_code(can_getGSTCode()))
		 {
			 ignition_order = can_getIgnitionOrder();
			 switch (ignition_order)
			 {
				case MAIN_IGNITION_ON: //Main Ignition On
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
						can_setFrame(GPIO_PIN_SET, DATA_ID_MAIN_IGNITION_STATE, HAL_GetTick());
					break;
				}
				case MAIN_IGNITION_OFF: //Main Ignition Off
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
						can_setFrame(GPIO_PIN_RESET, DATA_ID_MAIN_IGNITION_STATE, HAL_GetTick());
					break;
				}
				case SECONDARY_IGNITION_ON: //Secondary Ignition On
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
						can_setFrame(GPIO_PIN_SET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
					break;
				}
				case SECONDARY_IGNITION_OFF: //Secondary Ignition Off
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
						can_setFrame(GPIO_PIN_RESET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
					break;
				}
				case 0x00: //TEST
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET)
						can_setFrame(GPIO_PIN_SET, DATA_ID_MAIN_IGNITION_STATE, HAL_GetTick());
					break;
				}

			 }
			 disconnect_order = can_getOrder();
//			 if(disconnect_order == STATE_DISCONNECT_HOSE)
//			 {
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//				if((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) &&
//				   (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET))
//					can_setFrame(GPIO_PIN_SET, DATA_ID_HOSE_DISCONNECT_STATE, HAL_GetTick());
//			 }
		 }
		can_setFrame(0x22, DATA_ID_IGNITION, HAL_GetTick());

		//Read Current sensor
		//current = read_current();
#ifdef IGNITION_1
		can_setFrame(current, DATA_ID_IGNITION_CURRENT_1, HAL_GetTick());
#endif
#ifdef IGNITION_2
		can_setFrame(current, DATA_ID_IGNITION_CURRENT_2, HAL_GetTick());
#endif
#ifdef DISCONNECT
		can_setFrame(current, DATA_ID_DISCONNECT_CURRENT, HAL_GetTick());
#endif
		osDelay(1000);
	 }
}

float read_current()
{
	float current = 0;


	return current;
}

