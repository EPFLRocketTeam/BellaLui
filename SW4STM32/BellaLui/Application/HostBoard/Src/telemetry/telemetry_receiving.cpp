/*
 * telemetry_receiving.cpp
 *
 *  Created on: 19 Apr 2018
 *      Author: Clement Nussbaumer
 *      Alexandre Devienne
 *      ofacklam
 */

#include "telemetry/telemetry_receiving.h"
#include "telemetry/telemetry_protocol.h"

#include <stdbool.h>

extern "C" {
	#include "debug/console.h"
	#include "can_transmission.h"
}


// Received Packet Handling

bool telemetryReceivePropulsionCommand(uint32_t timestamp, uint8_t* payload) {
	uint8_t command = payload[0];

	can_setFrame(command, DATA_ID_PROP_COMMAND, timestamp);

	return 0;
}
