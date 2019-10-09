/*
 * CAN_handling.h
 *
 *  Created on: Jun 10, 2019
 *      Author: Alexandre Devienne
 */

#ifndef CAN_HANDLING_H_
#define CAN_HANDLING_H_

#include "Misc/datastructs.h"

void TK_can_reader();

float can_getAltitude();
float can_getSpeed();
uint8_t can_getState();
int32_t can_getABangle();

#endif /* CAN_HANDLING_H_ */
