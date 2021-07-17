#include "misc/state_machine_helpers.h"

#include "misc/rocket_constants.h"

namespace state_machine_helpers {

    uint8_t handleIdleState(const uint32_t currentTime, const uint32_t liftoff_time, const float acceleration_z) {
        bool liftoffAccelTrig = acceleration_z > ROCKET_CST_LIFTOFF_TRIG_ACCEL; //Compute lift-off triggers for acceleration

        if(liftoff_time == NO_LIFTOFF_TIME && liftoffAccelTrig){
            return state_idle_liftoff_detected;
        }
        else if (liftoff_time != NO_LIFTOFF_TIME) {
            if(!liftoffAccelTrig){
                return state_idle_false_positive;
            }
            //already detected the acceleration trigger. now we need the trigger for at least 1000ms before trigerring the liftoff.
            else if (currentTime - liftoff_time > LIFTOFF_DETECTION_DELAY) {
                return state_idle_switch_to_liftoff_state;
            }
        }
        return state_idle_no_op;
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

    uint8_t handleCoastState(const float max_altitude, const float baro_data_altitude, const float baro_data_base_altitude, const uint32_t apogee_counter) {
        // update the maximum altitude detected up to this point
        if (max_altitude < baro_data_altitude) {					
            return state_coast_rocket_is_ascending; 
        } 
        else {
            // if the number of measurements exceeds a certain value (basic noise filtering) then the altitude trigger based on the counter is enabled
            bool counterAltTrig = apogee_counter+1 > APOGEE_BUFFER_SIZE;
            // since the rocket is then supposed to be descending, the trigger waits for an altitude offset greater than the one defined to occure before triggering the state change
            bool diffAltTrig = (max_altitude - baro_data_altitude) > APOGEE_ALT_DIFF; 
            bool minAltTrig = ((baro_data_altitude - baro_data_base_altitude) > ROCKET_CST_MIN_TRIG_AGL);

            if (minAltTrig && counterAltTrig && diffAltTrig) {
                return state_coast_switch_to_primary_state;
            }
        }

        return state_coast_no_op;        
    }

    uint8_t handlePrimaryState(const uint32_t currentTime, const uint32_t time_tmp, const float baro_data_altitude, const float baro_data_base_altitude, const uint32_t sec_counter) {
        // update the minimum altitude detected up to this point
        if ((baro_data_altitude - baro_data_base_altitude) > ROCKET_CST_REC_SECONDARY_ALT) {
            return state_primary_altitude_above_secondary_altitude; // As long as the measured altitude is above the secondary recovery event altitude, keep buffer counter to 0
        } 
        else {
            bool counterSecTrig = sec_counter+1 > SECONDARY_BUFFER_SIZE; // if more than a given amount of measurements are below the secondary recovery altitude, toggle the state trigger
            
            bool sensorMuteTimeTrig = (currentTime - time_tmp) > APOGEE_MUTE_TIME; // check that some time has passed since the detection of the apogee before triggering the secondary recovery event

            if (counterSecTrig && sensorMuteTimeTrig)
            {
                return state_primary_switch_to_secondary_state;
            }
        }

        return state_primary_no_op;
    }

    float abs_fl32(float v) { // copied in order to avoid including common.h
	    return (v >= 0) ? v : -v;
    }

    uint8_t handleSecondaryState(const uint32_t currentTime, const uint32_t time_tmp, const bool baro_is_ready, const float baro_data_altitude, const float td_last_alt, const uint32_t td_counter){
        // if a given time has passed since the last time the check was done, do the check
        if (((currentTime - time_tmp) > TOUCHDOWN_DELAY_TIME) && baro_is_ready) {

                if (abs_fl32(baro_data_altitude - td_last_alt) <= TOUCHDOWN_ALT_DIFF){

                    if (td_counter+1 > TOUCHDOWN_BUFFER_SIZE){
                        return state_secondary_switch_to_touchdown_state;
                    }

                    return state_secondary_approaching_touchdown;
                }

                return state_secondary_altitude_difference_still_large;
            }

        return state_secondary_no_op;
    }

    uint8_t newImuDataIsAvailable(const uint32_t currentImuSeqNumber, const uint32_t lastImuSeqNumber) {
        return currentImuSeqNumber > lastImuSeqNumber;
    }

    uint8_t newBarometerDataIsAvailable(const uint32_t currentBaroSeqNumber, const uint32_t lastBaroSeqNumber){
        return currentBaroSeqNumber > lastBaroSeqNumber;
    }

    bool touchdownStateIsReached(const uint32_t currentTime, const uint32_t liftoff_time){
        const bool timeExceedsFiveMinutes = ((int32_t) currentTime - (int32_t) liftoff_time) > 5 * 60 * 1000;
        return liftoff_time != NO_LIFTOFF_TIME && timeExceedsFiveMinutes;
    }

}
