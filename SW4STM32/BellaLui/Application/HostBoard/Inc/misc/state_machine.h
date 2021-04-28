/*
 * state_machine.h
 *
 *  Created on: 27 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef MISC_STATE_MACHINE_H_
#define MISC_STATE_MACHINE_H_

#include <stdint.h>
#include <stdbool.h>

#include "datastructs.h"
#include "common.h"

class StateMachine {
public:
	StateMachine();
	~StateMachine() {};

	void publishState();
	void requestState(enum State new_state);
	void enterState(enum State new_state);
	enum State getCurrentState();

	IMU_data latestIMUData;
	BARO_data latestBarometerData;

	bool newIMUData;
	bool newBarometerData;

	uint32_t liftoffTime;

private:
	enum State current_state;
};


#endif /* MISC_STATE_MACHINE_H_ */
