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

	switch(command) {
	case 0x01:
		can_setFrame(0xC0FFEE, DATA_ID_START_VALVE_OPERATION, timestamp);
		break;
	case 0x02:
		can_setFrame(0xC0FFEE, DATA_ID_START_FUELING, timestamp);
		break;
	case 0x03:
		can_setFrame(0xC0FFEE, DATA_ID_STOP_FUELING, timestamp);
		break;
	case 0x04:
		can_setFrame(0xC0FFEE, DATA_ID_START_HOMING, timestamp);
		break;
	case 0x05:
		can_setFrame(0xC0FFEE, DATA_ID_ABORT, timestamp);
		break;
	default:
		rocket_boot_log("Invalid propulsion command received from GS: %d\n", command);
	}

	return 0;
}
