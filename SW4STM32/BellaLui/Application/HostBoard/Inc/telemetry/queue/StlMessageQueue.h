/*
 * StlMessageQueue.h
 *
 *  Created on: Feb 28, 2021
 *      Author: ofacklam
 */

#ifndef APPLICATION_HOSTBOARD_INC_TELEMETRY_QUEUE_STLMESSAGEQUEUE_H_
#define APPLICATION_HOSTBOARD_INC_TELEMETRY_QUEUE_STLMESSAGEQUEUE_H_

#include "telemetry/queue/AbstractMessageQueue.h"
#include <queue>

/**
 * BEWARE: this class is NOT THREAD-SAFE !! It is only meant for testing purposes.
 * It also doesn't respect the timeout constraints.
 */

template<typename T>
class StlMessageQueue : public AbstractMessageQueue<T> {
public:
	StlMessageQueue(uint32_t queueSize);

	virtual bool push(T *val, uint32_t timeout);
	virtual bool pop(T **dst, uint32_t timeout);

	uint32_t count();

private:
	std::queue<T*> q_;
};

template<typename T>
StlMessageQueue<T>::StlMessageQueue(uint32_t queueSize): AbstractMessageQueue<T>(queueSize) {
}

template<typename T>
bool StlMessageQueue<T>::push(T *val, uint32_t timeout) {
	if(q_.size() >= this->size_)
		return false;

	q_.push(val);
	return true;
}

template<typename T>
bool StlMessageQueue<T>::pop(T **dst, uint32_t timeout) {
	if(q_.size() <= 0)
		return false;

	*dst = q_.front();
	q_.pop();
	return true;
}

template<typename T>
uint32_t StlMessageQueue<T>::count() {
	return q_.size();
}

#endif /* APPLICATION_HOSTBOARD_INC_TELEMETRY_QUEUE_STLMESSAGEQUEUE_H_ */
