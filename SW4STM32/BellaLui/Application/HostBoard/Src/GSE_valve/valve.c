/*
 * valve.c
 *
 *  Created on: 1 jul. 2020
 *      Author: Lucas
 */

#include <GSE_valve/valve.h>
#include <stm32f446xx.h>
#include "stm32f4xx_hal.h"
#include <cmsis_os.h>
#include <misc/Common.h>
#include <debug/console.h>
#include <CAN_communication.h>
#include <CAN_handling.h>
#include <debug/led.h>


void valve_init(void)
{
	//Initialise S2 Socket GPIOs
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	rocket_log("Valves initialised. \n");

	//Set all valves to low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);


}
void TK_GSE_valve_control(void const * argument)
{
	//if xbee code = purge (D0)
	//if xbee code = purge backup (D1)
	//if xbee code = fill (D2)
	//if xbee code = fill backup (D3)

	uint32_t valve_order = 0;
	uint32_t current_state = 0;

	 for(;;) {
		 valve_order = can_getOrder();
		 switch (valve_order)
			{
				case STATE_OPEN_FILL_VALVE:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
					if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
						current_state = STATE_OPEN_FILL_VALVE;
					break;
				}
				case STATE_CLOSE_FILL_VALVE:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
					if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)
						current_state = STATE_CLOSE_FILL_VALVE;
					break;
				}
				case STATE_OPEN_PURGE_VALVE:
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET)
						current_state = STATE_OPEN_PURGE_VALVE;
					break;
				}
				case STATE_CLOSE_PURGE_VALVE:
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
					if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET)
						current_state = STATE_CLOSE_PURGE_VALVE;
					break;
				}
				case STATE_OPEN_FILL_VALVE_BACKUP:
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
						current_state = STATE_OPEN_FILL_VALVE_BACKUP;
					break;
				}
				case STATE_CLOSE_FILL_VALVE_BACKUP:
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
						current_state = STATE_CLOSE_FILL_VALVE_BACKUP;
					break;
				}
				case STATE_OPEN_PURGE_VALVE_BACKUP:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
					if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
						current_state = STATE_OPEN_PURGE_VALVE_BACKUP;
					break;
				}
				case STATE_CLOSE_PURGE_VALVE_BACKUP:
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
					if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET)
						current_state = STATE_CLOSE_PURGE_VALVE_BACKUP;
					break;
				}
			}
		 can_setFrame(current_state, DATA_ID_ORDER, HAL_GetTick());
		 rocket_log("New State sent over CAN \n");
		 osDelay(50);
	 }
}
