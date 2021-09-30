/* telemetry_receiving.h
 *
 *  Created on: 10 Jun 2019
 *      Author: Alexandre Devienne
 */

#ifndef TELEMETRY_RECEIVING_H_
#define TELEMETRY_RECEIVING_H_

#include <stdbool.h>

#include "../../../HostBoard/Inc/misc/datastructs.h"

#ifdef __cplusplus
extern "C" {
#endif

bool telemetryReceivePropulsionCommand(uint32_t timestamp, uint8_t* payload);
bool telemetryReceiveTVCCommand(uint32_t timestamp, uint8_t* payload);

#ifdef __cplusplus
}
#endif


#endif /* TELEMETRY_RECEIVING_H_ */
