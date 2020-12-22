#include "misc/state_machine_helpers.h"

#include "misc/rocket_constants.h"

namespace state_machine_helpers {

    bool handleLiftoffState(const uint32_t currentTime, const uint32_t previousTime) {
		// determine motor burn-out based on lift-off detection
		if ((currentTime - previousTime) > ROCKET_CST_MOTOR_BURNTIME) {
		    //return STATE_COAST; // switch to coast state
            return true;
        }
        //return STATE_LIFTOFF;
        return false;
    }

}