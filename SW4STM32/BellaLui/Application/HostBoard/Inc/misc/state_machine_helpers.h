#ifndef MISC_STATE_MACHINE_HELPERS_H_
#define MISC_STATE_MACHINE_HELPERS_H_

#include <stdint.h>

namespace state_machine_helpers {
    // TODO: change to Enum
    const uint8_t state_idle_false_positive = 11;
    const uint8_t state_idle_liftoff_detected = 12;
    const uint8_t state_idle_switch_to_liftoff_state = 13;

    const uint8_t state_coast_rocket_is_ascending = 21;
    const uint8_t state_coast_switch_to_primary_state = 22;

    const uint8_t state_primary_altitude_above_secondary_altitude = 31;
    const uint8_t state_primary_switch_to_secondary_state = 32;

    uint8_t handleIdleState(const uint32_t currentTime, const uint32_t liftoff_time, const float acceleration_z);
    bool handleLiftoffState(const uint32_t currentTime, const uint32_t previousTime);
    uint8_t handleCoastState(const float max_altitude, const float baro_data_altitude, const float baro_data_base_altitude, const uint32_t apogee_counter);
    uint8_t handlePrimaryState(const uint32_t currentTime, const uint32_t time_tmp, const float baro_data_altitude, const float baro_data_base_altitude, const uint32_t sec_counter);


    uint8_t newImuDataIsAvailable(const uint32_t currentImuSeqNumber, const uint32_t lastImuSeqNumber);
    uint8_t newBarometerDataIsAvailable(const uint32_t currentBaroSeqNumber, const uint32_t lastBaroSeqNumber);
    bool touchdownStateIsReached(const uint32_t currentTime, const uint32_t liftoff_time);
}

#endif /* MISC_STATE_MACHINE_HELPERS_H_ */