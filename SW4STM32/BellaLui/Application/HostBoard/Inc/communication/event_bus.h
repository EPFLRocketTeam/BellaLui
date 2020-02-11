/*
 * bus.h
 *
 *  Created on: 09 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_COMMUNICATION_EVENT_BUS_H_
#define APPLICATION_HOSTBOARD_INC_COMMUNICATION_EVENT_BUS_H_

#include <ETL/unordered_map.h>
#include <ETL/unordered_multimap.h>
#include <ETL/vector.h>
#include <stdint.h>

using namespace etl;

class EventBus {
public:
	template<typename T> void register_object(uint8_t packet_id);
	template<typename T> void register_handler(void (*handler)(T*));
	template<typename T> void send(T *packet);
	void receive(uint8_t *pointer, uint8_t length);


protected:
	virtual void write(uint8_t* buffer, uint8_t length) {} // Generic write function

private:
	unordered_map<size_t, uint8_t, 256> identifiers;
	unordered_map<uint8_t, uint8_t, 256> safe_cast_reference;
	unordered_multimap<uint8_t, void (*)(void*), 64> handlers;
};


#endif /* APPLICATION_HOSTBOARD_INC_COMMUNICATION_EVENT_BUS_H_ */
