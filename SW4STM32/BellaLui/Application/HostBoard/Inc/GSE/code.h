/*
 * code.h
 *
 *  Created on: 22 Oct 2020
 *      Author: lucas
 */

#ifndef APPLICATION_HOSTBOARD_INC_GSE_CODE_H_
#define APPLICATION_HOSTBOARD_INC_GSE_CODE_H_

void code_init(void);
void TK_code_control(void const * argument);

uint8_t verify_security_code(uint8_t GST_code);

#endif /* APPLICATION_HOSTBOARD_INC_GSE_CODE_H_ */
