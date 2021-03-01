#include "misc/state_machine_helpers.h"

#include "misc/rocket_constants.h"

namespace state_machine_helpers {

    uint8_t handleIdleState(const uint32_t currentTime, const uint32_t liftoff_time, uint8_t liftoffAccelTrig) {
        if(liftoff_time == 0 && liftoffAccelTrig){
            return state_idle_liftoff_detected;
        }
        else if (liftoff_time != 0) {
            if(!liftoffAccelTrig){
                return state_idle_false_positive;
            }
            //already detected the acceleration trigger. now we need the trigger for at least 1000ms before trigerring the liftoff.
            else if (currentTime - liftoff_time > LIFTOFF_DETECTION_DELAY) {
                return state_idle_switch_to_liftoff_state;
            }
        }
        return 0;
    }

    bool handleLiftoffState(const uint32_t currentTime, const uint32_t previousTime) {
		// determine motor burn-out based on lift-off detection
		if ((currentTime - previousTime) > ROCKET_CST_MOTOR_BURNTIME) {
		    //return STATE_COAST; // switch to coast state
            return true;
        }
        //return STATE_LIFTOFF;
        return false;
    }

    uint8_t newImuDataIsAvailable(const uint32_t currentImuSeqNumber, const uint32_t lastImuSeqNumber) {
        return currentImuSeqNumber > lastImuSeqNumber;
    }

    uint8_t newBarometerDataIsAvailable(const uint32_t currentBaroSeqNumber, const uint32_t lastBaroSeqNumber){
        return currentBaroSeqNumber > lastBaroSeqNumber;
    }

    bool touchdownStateIsReached(const uint32_t currentTime, const uint32_t liftoff_time){
        const bool timeExceedsFiveMinutes = ((int32_t) currentTime - (int32_t) liftoff_time) > 5 * 60 * 1000;
        return liftoff_time != 0 && timeExceedsFiveMinutes;
    }

}