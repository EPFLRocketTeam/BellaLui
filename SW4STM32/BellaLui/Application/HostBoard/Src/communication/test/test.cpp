/*
 * test.cpp
 *
 *  Created on: 12 Feb 2020
 *      Author: Arion
 */

#include <debug/console.h>

#include <communication/dispatcher.h>
#include <communication/event_bus.h>

void run_test1();


extern "C" void run_tests() {
	run_test1();

}

void run_test1() {
	DumpEvent event = {1, 2};
	EventBus bus;
	bus.send(&event);
}

