/*
 * airbrakes.c
 *
 *  Created on: 28 Apr 2019
 *      Author: Alexandre
 */

#include "stm32f4xx_hal.h"
#include <cmsis_os.h>

#include <airbrakes/airbrake.h>
#include <can_reception.h>
#include <can_transmission.h>
#include <misc/Common.h>
#include <debug/led.h>

#define AB_PERIOD_MS (50)

//#include "Airbrakes/controller_functions.h"
//#include <Misc/rocket_constants.h>

//extern volatile uint32_t flight_status;
int led_AB_id;

void TK_ab_controller(void const *argument) {
	osDelay(2000);

	led_AB_id = led_register_TK();

	while (!aerobrakes_control_init()) {
		led_set_TK_rgb(led_AB_id, 500, 50, 0);
		osDelay(1000);
	}

	led_set_TK_rgb(led_AB_id, 20, 50, 0);

	aerobrake_helloworld();

	osDelay(1000);

	while (true) {
		if (current_state < STATE_COAST) {
			full_close();
			led_set_TK_rgb(led_AB_id, 0, 10, 0);
		} else if (current_state == STATE_COAST) {
			command_aerobrake_controller(can_getAltitude(), can_getSpeed());
			led_set_TK_rgb(led_AB_id, 50, 50, 50);
		} else if (current_state >= STATE_PRIMARY) {
			full_close();
			led_set_TK_rgb(led_AB_id, 50, 50, 0);
		}

		osDelay(AB_PERIOD_MS);
	}
}
