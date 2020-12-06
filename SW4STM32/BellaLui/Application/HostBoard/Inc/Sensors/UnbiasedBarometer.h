/*
 * UnbiasedBarometer.h
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDBAROMETER_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDBAROMETER_H_

#include "Sensors/UnbiasedSensor.h"
#include "Sensors/Barometer.h"


class UnbiasedBarometer : public UnbiasedSensor<Barometer> {
public:
	void filterData(BarometerData* measurements, uint8_t count, BarometerData* output);
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDBAROMETER_H_ */
