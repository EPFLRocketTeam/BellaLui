/*
 * sensor_board.h
 *
 *  Created on: 21 Mar 2019
 *      Author: linky
 */

#ifndef SENSORS_SENSOR_BOARD_H_
#define SENSORS_SENSOR_BOARD_H_

#include "Misc/rocket_constants.h"
#include <math.h>

void TK_sensor_board(void const * argument);

inline float altitudeFromPressure(float pressure_hPa)
{
	return 44330 * (1.0 - pow (pressure_hPa / ADJUSTED_SEA_LEVEL_PRESSURE, 0.1903));
}

#endif /* SENSORS_SENSOR_BOARD_H_ */
