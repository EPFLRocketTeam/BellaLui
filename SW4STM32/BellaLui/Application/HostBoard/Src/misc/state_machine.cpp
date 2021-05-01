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
#include "debug/console.h"
#include "telemetry/telemetry_sending.h"
#include "sync.h"

#include <cmsis_os.h>
#include <misc/common.h>
#include <stm32f4xx_hal.h>


static const char *state_names[] = { "Sleeping", "Calibrating", "Idle",
		"Fill valve open", "Fill valve closed", "Purge valve open", "Hose disconnected",
		"Lift-off", "Coast", "Primary event", "Secondary event", "Touchdown" };



static StateMachine fsm;


StateMachine::StateMachine() :
		latestIMUData({0}),
		latestBarometerData({0}),
		newIMUData(false),
		newBarometerData(false),
		liftoffTime(-1) {
}


void StateMachine::requestState(enum State newState) {
	can_setFrame(newState, DATA_ID_STATE, HAL_GetTick());
}

void StateMachine::enterState(enum State new_state) {
	if(new_state >= 0 && new_state < NUM_STATES) {
		this->current_state = new_state;
		rocket_log("Entered %s state\r\n", state_names[new_state]);
	} else {
		rocket_log("Attempt was made to enter an inconsistent state (%d)\r\n", new_state);
	}
}


void TK_state_machine(void const *argument) {
	osDelay(2000);

	// Declare time variable
	uint32_t time_tmp = 0;

	// Declare sensor variables
	IMU_data imu_data = {0};
	BARO_data baro_data = {0};
	uint8_t imuIsReady = 0, baroIsReady = 0;

	// Declare apogee detection variables
	float max_altitude = 0;
	uint32_t apogee_counter = 0;

	// Declare secondary recovery event detection variables
	uint32_t sec_counter = 0;

	// Declare touch-down event detection variables
	float td_last_alt = 0;
	uint32_t td_counter = 0;

	uint32_t flight_status = 0;
	uint32_t preliminary_liftoff_time = 0;

	// TODO: Set low package data rate

	// State Machine initialization
	// Hyp: rocket is on rail waiting for lift-off
	fsm.requestState(STATE_CALIBRATION);


	// State Machine main task loop
	while(true) {
		sync_logic(10);

		imuIsReady = fsm.newIMUData;

		if (imuIsReady) {
			// Update accelerometer reading
			imu_data = fsm.latestIMUData;
			fsm.newIMUData = false;
		}

		baroIsReady = fsm.newBarometerData;

		if (baroIsReady) {
			// Update barometer reading
			baro_data = fsm.latestBarometerData;
			fsm.newBarometerData = false;
		}

		if(state_machine_helpers::touchdownStateIsReached(HAL_GetTick(), preliminary_liftoff_time)){
			fsm.requestState(STATE_TOUCHDOWN);
			flight_status = 40; // TODO: flight_status numbers should be defined as consts
		}


		enum State currentState = fsm.getCurrentState();

		if(enter_monitor(STATE_MONITOR) && currentState >= 0 && currentState < NUM_STATES) {
			rocket_log(" Time: %dms\x1b[K\n", HAL_GetTick());
			rocket_log(" Current state: %s\x1b[K\n", state_names[currentState]);

			if(baroIsReady) {
				rocket_log(" Altitude: %d\x1b[K\n", (int32_t) (1000 * (baro_data.altitude - baro_data.base_altitude)));
			} else {
				rocket_log(" Altitude unavailable\x1b[K\n");
			}

			exit_monitor(STATE_MONITOR);
		}

		// State Machine
		switch (currentState) {

		case STATE_CALIBRATION: {
			if (baroIsReady) {
				fsm.requestState(STATE_IDLE);
			}
			break;
		}

		case STATE_IDLE: {
			if (imuIsReady) {
				const uint32_t currentTime = HAL_GetTick();
				uint8_t state_idle_status = state_machine_helpers::handleIdleState(currentTime, preliminary_liftoff_time, abs_fl32(imu_data.acceleration.z));

				if(state_idle_status == state_machine_helpers::state_idle_false_positive){
					preliminary_liftoff_time = 0;
					time_tmp = 0;
				}
				else if (state_idle_status == state_machine_helpers::state_idle_liftoff_detected){
					preliminary_liftoff_time = currentTime;
					time_tmp = currentTime; // Start timer to estimate motor burn out
				}
				else if(state_idle_status == state_machine_helpers::state_idle_switch_to_liftoff_state) {
					fsm.liftoffTime = preliminary_liftoff_time;
					fsm.requestState(STATE_LIFTOFF); // Switch to lift-off state
				}
			}
			break;
		}

		case STATE_LIFTOFF: {
			flight_status = 10; // TODO: flight_status numbers should be defined as consts

			if(state_machine_helpers::handleLiftoffState(HAL_GetTick(), time_tmp)){
				fsm.requestState(STATE_COAST);
			}

			break;
		}

		case STATE_COAST: {
			flight_status = 20;

			// compute apogee triggers for altitude
			if (baroIsReady) {
				uint8_t state_coast_status = state_machine_helpers::handleCoastState(max_altitude, baro_data.altitude, baro_data.base_altitude, apogee_counter);

				if(state_coast_status == state_machine_helpers::state_coast_rocket_is_ascending){
					max_altitude = baro_data.altitude; // if a new maximum altitude is found, this means the rocket is going up
					apogee_counter = 0; // The descending measurement counter is thus set to zero
				}
				else{
					++apogee_counter; // if the rocket isn't rising then it is descending, thus the number of descending measurements are counted
					if(state_coast_status == state_machine_helpers::state_coast_switch_to_primary_state){
						time_tmp = HAL_GetTick(); // save time to mute sensors while ejection occures
						fsm.requestState(STATE_PRIMARY); // switch to primary descent phase
						flight_status = 30; // TODO: flight_status numbers should be defined as consts
					}
				}
			}
			break;
		}

		case STATE_PRIMARY: {
			if (baroIsReady) {
				uint8_t state_primary_status = state_machine_helpers::handlePrimaryState(HAL_GetTick(), time_tmp, baro_data.altitude, baro_data.base_altitude, sec_counter);

				if(state_primary_status == state_machine_helpers::state_primary_altitude_above_secondary_altitude){
					sec_counter = 0;
				}
				else{
					++sec_counter; // if the measured altitude is lower than the trigger altitude, start counting
					if(state_primary_status == state_machine_helpers::state_primary_switch_to_secondary_state){
						time_tmp = HAL_GetTick(); // save current time to start differed touchdown detection rate
						fsm.requestState(STATE_SECONDARY); // switch to secondary recovery phase
						td_last_alt = baro_data.altitude; // save altitude measurement for touchdown detection
						flight_status = 35; // TODO: flight_status numbers should be defined as consts
					}
				}
			}
			break;
		}

		case STATE_SECONDARY: {
			const float baro_data_altitude = baroIsReady ? baro_data.altitude : -1;

			uint8_t state_secondary_status = state_machine_helpers::handleSecondaryState(HAL_GetTick(), time_tmp, baroIsReady, baro_data_altitude, td_last_alt, td_counter);

			if(state_secondary_status != state_machine_helpers::state_secondary_no_op){
				time_tmp = HAL_GetTick();
				td_last_alt = baro_data.altitude;

				if(state_secondary_status == state_machine_helpers::state_secondary_altitude_difference_still_large){
					td_counter = 0;
				}
				else{
					++td_counter;
					if(state_secondary_status == state_machine_helpers::state_secondary_switch_to_touchdown_state){
						fsm.requestState(STATE_TOUCHDOWN);
						flight_status = 40; // TODO: flight_status numbers should be defined as consts
						// TODO: Set telemetry data rate to low
					}
				}
			}

			break;
		}

		case STATE_TOUCHDOWN:
			break;

		default:
			// Error handling
			break;
		}
	}
}

extern "C" void onIMUDataReception(IMU_data data) {
	fsm.latestIMUData = data;
	fsm.newIMUData = true;
}

extern "C" void onBarometerDataReception(BARO_data data) {
	fsm.latestBarometerData = data;
	fsm.newBarometerData = true;
}

extern "C" void onStateAcknowledged(enum State newState) {
	fsm.enterState(newState);
}

extern "C" enum State getAvionicsState() {
	return fsm.getCurrentState();
}

extern "C" int32_t getLiftoffTime() {
	return fsm.liftoffTime;
}
