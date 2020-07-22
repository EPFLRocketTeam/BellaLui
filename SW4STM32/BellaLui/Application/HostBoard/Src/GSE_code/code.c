/*
 * code.c
 *
 *  Created on: 8 Jul 2020
 *      Author: lucaspallez
 */

#include <cmsis_os.h>
#include <CAN_communication.h>
#include <CAN_handling.h>
#include <debug/console.h>
#include <GSE_code/code.h>
#include <stm32_hal_legacy.h>
#include <stm32f446xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <sys/_stdint.h>


#define CODE_SIZE 4

void code_init(void)
{
	//Initialise all GPIO inputs on S2 socket
//	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.Pin = GPIO_PIN_15;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_7;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//	GPIO_InitStruct.Pin = GPIO_PIN_13;
//	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	//Initialise S1 Socket GPIOs
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	rocket_log("Security code initialised.\n");
}


void TK_code_control(void const * argument)
{
	int code[CODE_SIZE];
	uint32_t code_int;
	uint32_t GST_code;

	for(;;)
	{
		//S2
//		code[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15); //Read D0
//		code[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9); //Read D1
//		code[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7); //Read D2
//		code[3] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); //Read D3

		//S1
		code[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); //Read D0
		code[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1); //Read D1
		code[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11); //Read D2
		code[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12); //Read D3
		//Convert into single int
		code_int = 0;
		for (int i = 0; i < CODE_SIZE; i++)
		    code_int = 10 * code_int + code[i];

		can_setFrame(code_int, DATA_ID_GSE_CODE, HAL_GetTick());
		osDelay(50);
	}
}
