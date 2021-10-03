#ifndef MISC_STATE_MACHINE_HELPERS_H_
#define MISC_STATE_MACHINE_HELPERS_H_

#include <stdint.h>

#define NO_LIFTOFF_TIME 0

namespace state_machine_helpers {
    // TODO: change to Enum
    const uint8_t state_idle_false_positive = 11;
    const uint8_t state_idle_liftoff_detected = 12;
    const uint8_t state_idle_switch_to_liftoff_state = 13;
    const uint8_t state_idle_no_op = 14;

    const uint8_t state_coast_rocket_is_ascending = 21;
    const uint8_t state_coast_switch_to_primary_state = 22;
    const uint8_t state_coast_no_op = 23;

    const uint8_t state_primary_altitude_above_secondary_altitude = 31;
    const uint8_t state_primary_switch_to_secondary_state = 32;
    const uint8_t state_primary_no_op = 33;

    const uint8_t state_secondary_altitude_difference_still_large = 41;
    const uint8_t state_secondary_approaching_touchdown = 42;
    const uint8_t state_secondary_switch_to_touchdown_state = 43;
    const uint8_t state_secondary_no_op = 44;


    // TODO: discuss use of acceleration_z vs. acceleration norm for this detection
    uint8_t handleIdleState(const uint32_t currentTime, const uint32_t liftoff_time, const float acceleration_z);
    // TODO: capture TVC's stop motor command for this
    bool handleLiftoffState(const uint32_t currentTime, const uint32_t previousTime);
    uint8_t handleCoastState(const float max_altitude, const float baro_data_altitude, const float baro_data_base_altitude, const uint32_t apogee_counter);
    uint8_t handlePrimaryState(const uint32_t currentTime, const uint32_t time_tmp, const float baro_data_altitude, const float baro_data_base_altitude, const uint32_t sec_counter);
    uint8_t handleSecondaryState(const uint32_t currentTime, const uint32_t time_tmp, const bool baro_is_ready, const float baro_data_altitude, const float td_last_alt, const uint32_t td_counter);

    uint8_t newImuDataIsAvailable(const uint32_t currentImuSeqNumber, const uint32_t lastImuSeqNumber);
    uint8_t newBarometerDataIsAvailable(const uint32_t currentBaroSeqNumber, const uint32_t lastBaroSeqNumber);
    bool touchdownStateIsReached(const uint32_t currentTime, const uint32_t liftoff_time);
}

#endif /* MISC_STATE_MACHINE_HELPERS_H_ */
