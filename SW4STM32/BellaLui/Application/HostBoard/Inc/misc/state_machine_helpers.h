#ifndef MISC_STATE_MACHINE_HELPERS_H_
#define MISC_STATE_MACHINE_HELPERS_H_

#include <stdint.h>

namespace state_machine_helpers {
    // TODO: change to Enum
    const uint8_t state_idle_false_positive = 101;
    const uint8_t state_idle_liftoff_detected = 102;
    const uint8_t state_idle_switch_to_liftoff_state = 103;

    const uint8_t state_coast_rocket_is_ascending = 201;
    const uint8_t state_coast_switch_to_primary_state = 202;

    uint8_t handleIdleState(const uint32_t currentTime, const uint32_t liftoff_time, uint8_t liftoffAccelTrig);
    bool handleLiftoffState(const uint32_t currentTime, const uint32_t previousTime);
    uint8_t handleCoastState(const float max_altitude, const float baro_data_altitude, const float baro_data_base_altitude, const uint32_t apogee_counter);
    
    uint8_t newImuDataIsAvailable(const uint32_t currentImuSeqNumber, const uint32_t lastImuSeqNumber);
    uint8_t newBarometerDataIsAvailable(const uint32_t currentBaroSeqNumber, const uint32_t lastBaroSeqNumber);
    bool touchdownStateIsReached(const uint32_t currentTime, const uint32_t liftoff_time);
}

#endif /* MISC_STATE_MACHINE_HELPERS_H_ */