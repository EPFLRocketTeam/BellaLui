/*
 * UnbiasedBarometer.h
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDBAROMETER_H_
#define APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDBAROMETER_H_

#include "Sensors/UnbiasedSensor.h"



class UnbiasedBarometer : public UnbiasedSensor<BarometerData> {
public:
	UnbiasedBarometer(const char* identifier, std::initializer_list<Sensor<BarometerData>*> sensors);

protected:
	uint16_t filterData(BarometerData* measurements, uint8_t count, BarometerData* output);
};


#endif /* APPLICATION_HOSTBOARD_INC_SENSORS_UNBIASEDBAROMETER_H_ */
