#ifndef MISC_STATE_MACHINE_HELPERS_H_
#define MISC_STATE_MACHINE_HELPERS_H_

#include <stdint.h>

namespace state_machine_helpers {
    bool handleLiftoffState(const uint32_t currentTime, const uint32_t previousTime);
    uint8_t newImuDataIsAvailable(const uint32_t currentImuSeqNumber, const uint32_t lastImuSeqNumber);
    uint8_t newBarometerDataIsAvailable(const uint32_t currentBaroSeqNumber, const uint32_t lastBaroSeqNumber);
    bool touchdownStateIsReached(const uint32_t currentTime, const uint32_t liftoff_time);
}

#endif /* MISC_STATE_MACHINE_HELPERS_H_ */