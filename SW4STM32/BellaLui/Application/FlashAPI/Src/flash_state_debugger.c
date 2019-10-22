/*
 * led.c
 *
 *  Created on: 5 Oct 2019
 *      Author: Arion
 */


#include <flash_state.h>
#include "tim.h"
#include "gpio.h"


#define TIMER TIM8
#define FATAL_PULSE_DELAY 100

#ifdef DEBUG

void __set_color(uint8_t r, uint8_t g, uint8_t b) {
	LL_TIM_OC_SetCompareCH1(TIMER, r);
	LL_TIM_OC_SetCompareCH2(TIMER, g);
	LL_TIM_OC_SetCompareCH3(TIMER, b);
}

void __led_init() {
	LL_TIM_CC_EnableChannel(TIMER, LL_TIM_CHANNEL_CH1N);
	LL_TIM_CC_EnableChannel(TIMER, LL_TIM_CHANNEL_CH2N);
	LL_TIM_CC_EnableChannel(TIMER, LL_TIM_CHANNEL_CH3N);
	LL_TIM_EnableAutomaticOutput(TIMER);
	LL_TIM_EnableCounter(TIMER);

	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15);

	__set_color(0, 0, 0);
}


void flash_init_state_debugger() {
	__led_init();
}


void flash_ready() {
	__set_color(0, 255, 255);
}

void flash_success() {
	__set_color(0, 255, 0);
}

void flash_error() {
	__set_color(255, 31, 0);
}

void flash_fatal(uint32_t error_code) {
	uint32_t error_reader = 0;

	while(1) {
		/*
		 * Since the error code represents a rhythm,
		 * a kind of iteration over the uint32 was necessary.
		 * I thus opted for a bitwise rotation of a reader bit mask,
		 * that indicates which bit must be read.
		 */

		error_reader >>= 1;

		if(!error_reader) {
			error_reader = 1 << 31;
		}

		if(error_code & error_reader) {
			__set_color(255, 0, 0);
		} else {
			__set_color(0, 0, 0);
		}


		HAL_Delay(FATAL_PULSE_DELAY);
	}
}

#else

void flash_init_state_debugger() {}

void flash_ready() {}
void flash_success() {}
void flash_error() {}
void flash_fatal(uint32_t error_code) {}

#endif
