/*
 * AbstractMessageQueue.h
 *
 *  Created on: Feb 28, 2021
 *      Author: ofacklam
 */

#ifndef APPLICATION_HOSTBOARD_SRC_TELEMETRY_ABSTRACTMESSAGEQUEUE_H_
#define APPLICATION_HOSTBOARD_SRC_TELEMETRY_ABSTRACTMESSAGEQUEUE_H_

#include <stdint.h>

template <typename T>
class AbstractMessageQueue {
public:
	AbstractMessageQueue(uint32_t queueSize) : size_(queueSize) {}
	virtual ~AbstractMessageQueue();

	virtual bool push(T *val, uint32_t timeout) = 0;
	virtual bool pop(T **dst, uint32_t timeout) = 0;

protected:
	uint32_t size_;
};

#endif /* APPLICATION_HOSTBOARD_SRC_TELEMETRY_ABSTRACTMESSAGEQUEUE_H_ */
