/*
 * dispatcher.h
 *
 *  Created on: 12 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_COMMUNICATION_DISPATCHER_H_
#define APPLICATION_HOSTBOARD_INC_COMMUNICATION_DISPATCHER_H_

#include <communication/protocol/protocol.h>


template<typename T> void dispatch(T *packet);


#endif /* APPLICATION_HOSTBOARD_INC_COMMUNICATION_DISPATCHER_H_ */
