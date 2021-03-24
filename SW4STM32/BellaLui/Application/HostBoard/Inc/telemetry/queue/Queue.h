/*
 * telemetry_queue.h
 *
 *  Created on: 24 Mar 2021
 *      Author: arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_TELEMETRY_TELEMETRY_QUEUE_H_
#define APPLICATION_HOSTBOARD_INC_TELEMETRY_TELEMETRY_QUEUE_H_


#include "AbstractMessageQueue.h"

void registerSendQueue(AbstractMessageQueue<Telemetry_Message> *queue);



#endif /* APPLICATION_HOSTBOARD_INC_TELEMETRY_TELEMETRY_QUEUE_H_ */
