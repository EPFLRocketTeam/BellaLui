/*
 * GPS_board.h
 *
 *  Created on: 12 Apr 2019
 *      Author: linky
 */

#ifndef GPS_BOARD_H_
#define GPS_BOARD_H_

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gps_init(UART_HandleTypeDef *gpsHuart);
void TK_GPS_board(void const * argument);

void GPS_RxCpltCallback ();
UART_HandleTypeDef* gps_gethuart();

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_GPS_BOARD_H_ */
