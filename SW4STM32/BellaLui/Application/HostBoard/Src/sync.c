/*
 * sync.c
 *
 *  Created on: 16 Feb 2020
 *      Author: Arion
 */

#include <debug/console.h>
#include <tim.h>
#include <stdint.h>


#define TICK_FREQUENCY 20


static TIM_HandleTypeDef the_timer;

static uint32_t last_update = 0;


void init_timer() {
	HAL_TIM_Base_MspInit(&htim2);

	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = -1;
	htim2.Init.Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.RepetitionCounter = 0;

	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start(&htim2);
}

uint32_t prev = 0;

void sync_data_acquisition() {
	uint32_t time = __HAL_TIM_GET_COUNTER(&htim2);
	int32_t delta = time - last_update;

	if(HAL_GetTick() - prev >= 100) {
		prev = HAL_GetTick();
		last_update = time;
		printf("%ld\n", delta);
	}

}
