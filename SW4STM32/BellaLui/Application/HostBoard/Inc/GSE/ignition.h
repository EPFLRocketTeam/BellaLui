/*
 * ignition.h
 *
 *  Created on: 22 Oct 2020
 *      Author: lucas
 */

#ifndef APPLICATION_HOSTBOARD_INC_GSE_IGNITION_H_
#define APPLICATION_HOSTBOARD_INC_GSE_IGNITION_H_

void ignition_sys_init(void);
void TK_ignition_control(void const * argument);
float read_current();


#endif /* APPLICATION_HOSTBOARD_INC_GSE_IGNITION_H_ */
