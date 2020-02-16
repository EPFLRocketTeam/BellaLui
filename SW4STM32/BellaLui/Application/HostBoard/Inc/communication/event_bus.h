/*
 * event_bus.h
 *
 *  Created on: 12 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_COMMUNICATION_EVENT_BUS_H_
#define APPLICATION_HOSTBOARD_INC_COMMUNICATION_EVENT_BUS_H_

#include "message_bus.h"

class EventBus : public MessageBus {
public:
	static EventBus instance;
protected:
	void init_protocol();
	void write(uint8_t* buffer, uint8_t length);
};


#endif /* APPLICATION_HOSTBOARD_INC_COMMUNICATION_EVENT_BUS_H_ */
