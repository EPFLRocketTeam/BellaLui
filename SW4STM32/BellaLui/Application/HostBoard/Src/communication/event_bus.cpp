/*
 * event_bus.cpp
 *
 *  Created on: 12 Feb 2020
 *      Author: Arion
 */

#include <debug/console.h>

#include <communication/event_bus.h>
#include <communication/protocol/protocol.h>

void EventBus::init_protocol() {
	//MessageBus::register_object<DumpEvent>(0);

}

void EventBus::write(uint8_t* buffer, uint8_t length) {
	receive(buffer, length); // Auto-redirect
}
