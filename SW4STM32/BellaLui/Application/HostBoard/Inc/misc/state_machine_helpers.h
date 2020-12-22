#ifndef MISC_STATE_MACHINE_HELPERS_H_
#define MISC_STATE_MACHINE_HELPERS_H_

#include <stdint.h>

namespace state_machine_helpers {
    bool handleLiftoffState(const uint32_t currentTime, const uint32_t previousTime);
}

#endif /* MISC_STATE_MACHINE_HELPERS_H_ */