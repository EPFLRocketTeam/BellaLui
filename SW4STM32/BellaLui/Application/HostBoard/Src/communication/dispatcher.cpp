/*
 * dispatcher.cpp
 *
 *  Created on: 12 Feb 2020
 *      Author: Arion
 */


#include <communication/dispatcher.h>
#include <communication/event_bus.h>

template<typename T> void dispatch(T *packet) {
	EventBus::instance.send(packet);
}
