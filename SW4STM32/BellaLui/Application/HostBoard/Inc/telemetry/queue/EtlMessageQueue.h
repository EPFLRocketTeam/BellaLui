/*
 * EtlMessageQueue.h
 *
 *  Created on: Feb 28, 2021
 *      Author: ofacklam
 */

#ifndef APPLICATION_HOSTBOARD_INC_TELEMETRY_QUEUE_ETLMESSAGEQUEUE_H_
#define APPLICATION_HOSTBOARD_INC_TELEMETRY_QUEUE_ETLMESSAGEQUEUE_H_

#include "telemetry/queue/AbstractMessageQueue.h"
#include <cmsis_os.h>

template<typename T>
class EtlMessageQueue : public AbstractMessageQueue<T> {
public:
	EtlMessageQueue(uint32_t queueSize, char *description);
	virtual ~EtlMessageQueue();

	virtual bool push(T *val, uint32_t timeout);
	virtual bool pop(T **dst, uint32_t timeout);

private:
	osMessageQId queueHandle;
};

template<typename T>
EtlMessageQueue<T>::EtlMessageQueue(uint32_t queueSize, char *description) : AbstractMessageQueue<T>(queueSize) {
	osMessageQDef(queue, queueSize, T*);
	queueHandle = osMessageCreate(osMessageQ(queue), NULL);
	vQueueAddToRegistry(queueHandle, description);
}

template<typename T>
bool EtlMessageQueue<T>::push(T *val, uint32_t timeout) {
	return osOK == osMessagePut(queueHandle, (uint32_t) val, timeout);
}

template<typename T>
bool EtlMessageQueue<T>::pop(T **dst, uint32_t timeout) {
	osEvent event = osMessageGet(queueHandle, timeout);

	if(event.status == osEventMessage) {
		*dst = (T*) event.value.p;
		return true;
	}

	return false;
}

#endif /* APPLICATION_HOSTBOARD_INC_TELEMETRY_QUEUE_ETLMESSAGEQUEUE_H_ */
