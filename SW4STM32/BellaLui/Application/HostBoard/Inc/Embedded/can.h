/*
 * can.h
 *
 *  Created on: 15 Mar 2021
 *      Author: ofacklam
 */

#ifndef APPLICATION_HOSTBOARD_SRC_EMBEDDED_CAN_H_
#define APPLICATION_HOSTBOARD_SRC_EMBEDDED_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef TESTING
#define can_getSpeed(x) (0)
#define can_getAltitude(x) (0)
#else
#include <can_reception.h>
#endif

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_HOSTBOARD_SRC_EMBEDDED_CAN_H_ */
