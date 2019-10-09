#include "Status_panel.h"


GPIO_TypeDef *Port_buzzer;
uint16_t Pin_buzzer;

GPIO_TypeDef* Port_led[4];
uint16_t Pin_led[4];

void Status_panel_init(uint32_t slot)
{
	if(slot == 1)
	{
		Port_buzzer = GPIOA;
		Pin_buzzer = GPIO_PIN_11;

		Port_led[0] = GPIOA;
		Pin_led[0] = GPIO_PIN_0;

		Port_led[1] = GPIOA;
		Pin_led[1] = GPIO_PIN_1;

		Port_led[2] = GPIOA;
		Pin_led[2] = GPIO_PIN_3;

		Port_led[3] = GPIOB;
		Pin_led[3] = GPIO_PIN_12;
	}

	if(slot == 2)
	{
		Port_buzzer = GPIOB;
		Pin_buzzer = GPIO_PIN_7;

		Port_led[0] = GPIOA;
		Pin_led[0] = GPIO_PIN_15;

		Port_led[1] = GPIOB;
		Pin_led[1] = GPIO_PIN_9;

		Port_led[2] = GPIOA;
		Pin_led[2] = GPIO_PIN_2;

		Port_led[3] = GPIOC;
		Pin_led[3] = GPIO_PIN_13;

	}

}



void setBuzzerPanel(uint32_t timeOn)
{

	    HAL_GPIO_WritePin(Port_buzzer, Pin_buzzer, SET);
	    osDelay(timeOn);
	    HAL_GPIO_WritePin(Port_buzzer, Pin_buzzer, RESET);
}

void setLedPanel(uint32_t Led_number)
{

	if(Led_number>=1 && Led_number<=4)
	{
		HAL_GPIO_WritePin(Port_led[Led_number-1], Pin_led[Led_number-1], SET);
	}

}

void resetLedPanel(uint32_t Led_number)
{
	if(Led_number>=1 && Led_number<=4)
	{
		HAL_GPIO_WritePin(Port_led[Led_number-1], Pin_led[Led_number-1], RESET);
	}

}


