/*
 * state_machine.c
 *
 *  Created on: 24 Apr 2018
 *      Author: ebrunner
 */

#include "misc/state_machine.h"
#include "misc/state_machine_helpers.h"
#include "can_transmission.h"
#include "misc/rocket_constants.h"
#include "debug/monitor.h"
#include "sync.h"

#include <cmsis_os.h>
#include <misc/common.h>
#include <stm32f4xx_hal.h>


void TK_state_machine(void const *argument) {
	current_state = STATE_SLEEP;

	osDelay(2000);

	// Declare time variable
	uint32_t time_tmp = 0;

	// Declare sensor variables
	IMU_data *imu_data = 0;
	BARO_data *baro_data = 0;
	uint32_t lastImuSeqNumber = 0, lastBaroSeqNumber = 0;
	uint8_t imuIsReady = 0, baroIsReady = 0;

	// Declare apogee detection variables
	float32_t max_altitude = 0;
	uint32_t apogee_counter = 0;

	// Declare secondary recovery event detection variables
	uint32_t sec_counter = 0;

	// Declare touch-down event detection variables
	float32_t td_last_alt = 0;
	uint32_t td_counter = 0;

	// TODO: Set low package data rate

	// State Machine initialization
	// Hyp: rocket is on rail waiting for lift-off
	current_state = STATE_CALIBRATION;

	// State Machine main task loop
	while(true) {
		sync_logic(10);
		can_setFrame(current_state, DATA_ID_STATE, HAL_GetTick());

		imuIsReady = state_machine_helpers::newImuDataIsAvailable(currentImuSeqNumber, lastImuSeqNumber);
		if (imuIsReady) {
			// Update accelerometer reading
			imu_data = getCurrentIMU_data();
			lastImuSeqNumber = currentImuSeqNumber;
		}

		baroIsReady = state_machine_helpers::newBarometerDataIsAvailable(currentBaroSeqNumber, lastBaroSeqNumber);
		if (baroIsReady) {
			// Update barometer reading
			baro_data = getCurrentBARO_data();
			lastBaroSeqNumber = currentBaroSeqNumber;
		}

		if(state_machine_helpers::touchdownStateIsReached(HAL_GetTick(), liftoff_time)){
			current_state = STATE_TOUCHDOWN;
			flight_status = 40; // TODO: flight_status numbers should be defined as consts
		}

		if(enter_monitor(STATE_MONITOR) && current_state < NUM_STATES) {
			rocket_log(" Time: %dms\x1b[K\n", HAL_GetTick());
			rocket_log(" Current state: %s\x1b[K\n", state_names[current_state]);
			if (baro_data != 0) {
				rocket_log(" Altitude: %d\x1b[K\n", (int32_t) (1000 * (baro_data->altitude - baro_data->base_altitude)));
			} else {
				rocket_log(" Altitude unavailable\x1b[K\n");
			}
			exit_monitor(STATE_MONITOR);
		}

		// State Machine
		switch (current_state) {

		case STATE_CALIBRATION: {
			if (baroIsReady) {
				current_state = STATE_IDLE;
			}
			break;
		}

		case STATE_IDLE: {
			if (imuIsReady) {
				const uint32_t currentTime = HAL_GetTick();
				// Compute lift-off triggers for acceleration
				uint8_t liftoffAccelTrig = (abs_fl32(imu_data->acceleration.z) > ROCKET_CST_LIFTOFF_TRIG_ACCEL);

				uint8_t state_idle_status = state_machine_helpers::handleIdleState(currentTime, liftoff_time, liftoffAccelTrig);

				if(state_idle_status == state_machine_helpers::state_idle_false_positive){
					liftoff_time = 0;
					time_tmp = 0;
				}
				else if (state_idle_status == state_machine_helpers::state_idle_liftoff_detected){
					liftoff_time = currentTime;
					time_tmp = currentTime; // Start timer to estimate motor burn out
				}
				else if(state_idle_status == state_machine_helpers::state_idle_switch_to_liftoff_state){
					current_state = STATE_LIFTOFF; // Switch to lift-off state
				}
			}
			break;
		}

		case STATE_LIFTOFF: {
			flight_status = 10; // TODO: flight_status numbers should be defined as consts
			current_state = state_machine_helpers::handleLiftoffState(HAL_GetTick(), time_tmp) ? STATE_COAST : STATE_LIFTOFF;
			break;
		}

		case STATE_COAST: {
			flight_status = 20;

			// compute apogee triggers for altitude
			if (baroIsReady) {
				uint8_t minAltTrig = ((baro_data->altitude - baro_data->base_altitude) > ROCKET_CST_MIN_TRIG_AGL);
				uint8_t counterAltTrig = 0;
				uint8_t diffAltTrig = 0;

				// update the maximum altitude detected up to this point
				if (max_altitude < baro_data->altitude) {
					// if a new maximum altitude is found, this means the rocket is going up
					max_altitude = baro_data->altitude;
					// The descending measurement counter is thus set to zero
					apogee_counter = 0;
				} else {
					// if the rocket isn't rising then it is descending, thus the number of descending measurements are counted
					apogee_counter++;
					if (apogee_counter > APOGEE_BUFFER_SIZE)
					// if the number of measurements exceeds a certain value (basic noise filtering)...
					{
						// ... then the altitude trigger based on the counter is enabled
						counterAltTrig = 1;
						if ((max_altitude - baro_data->altitude) > APOGEE_ALT_DIFF)
						// since the rocket is then supposed to be descending, the trigger waits for an altitude offset greater than the one defined to occure before triggering the state change
						{
							diffAltTrig = 1;
						}
					}
				}

				// detect apogee
				if (minAltTrig && counterAltTrig && diffAltTrig) {
					time_tmp = HAL_GetTick(); // save time to mute sensors while ejection occures
					current_state = STATE_PRIMARY; // switch to primary descent phase
					flight_status = 30;
				}
			}
			break;
		}

		case STATE_PRIMARY: {
			if (baroIsReady) {
				// check that some time has passed since the detection of the apogee before triggering the secondary recovery event
				uint8_t sensorMuteTimeTrig = ((HAL_GetTick() - time_tmp) > APOGEE_MUTE_TIME);
				uint8_t counterSecTrig = 0;

				// update the minimum altitude detected up to this point
				if ((baro_data->altitude - baro_data->base_altitude) > ROCKET_CST_REC_SECONDARY_ALT) {
					// As long as the measured altitude is above the secondary recovery event altitude, keep buffer counter to 0
					sec_counter = 0;
				} else {
					// if the measured altitude is lower than the trigger altitude, start counting
					sec_counter++;
					if (sec_counter > SECONDARY_BUFFER_SIZE)
					// if more than a given amount of measurements are below the secondary recovery altitude, toggle the state trigger
					{
						counterSecTrig = 1;
					}
				}

				//detect secondary recovery event
				if (sensorMuteTimeTrig && counterSecTrig) {
					time_tmp = HAL_GetTick(); // save current time to start differed touchdown detection rate
					current_state = STATE_SECONDARY; // switch to secondary recovery phase
					td_last_alt = baro_data->altitude; // save altitude measurement for touchdown detection
					flight_status = 35;
				}
			}
			break;
		}

		case STATE_SECONDARY: {
			uint8_t counterTdTrig = 0;

			// if a given time has passed since the last time the check was done, do the check
			if ((HAL_GetTick() - time_tmp) > TOUCHDOWN_DELAY_TIME) {
				if (baroIsReady) {
					// save time of check
					time_tmp = HAL_GetTick();

					// check if the altitude hasn't varied of more than a given amount since last time
					if (abs_fl32(baro_data->altitude - td_last_alt) > TOUCHDOWN_ALT_DIFF) {
						// if the altitude difference is still large, this means the rocket is descending and so the touch down counter is kept to zero
						td_counter = 0;
					} else {
						// if the altitude difference is in bounds, the counter is incremented
						td_counter++;
						if (td_counter > TOUCHDOWN_BUFFER_SIZE) {
							// if the counter is larger than a given value, toggle the state trigger
							counterTdTrig = 1;
						}
					}
					td_last_alt = baro_data->altitude;

					if (counterTdTrig) {
						current_state = STATE_TOUCHDOWN;
						flight_status = 40;
						// TODO: Set telemetry data rate to low
					}
				}
			}

			break;
		}

		case STATE_TOUCHDOWN:
			break;
		}

	}
}
