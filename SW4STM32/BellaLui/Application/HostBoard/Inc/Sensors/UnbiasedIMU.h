/*
 * UnbiasedIMU.h
 *
 *  Created on: 6 Dec 2020
 *      Author: Arion
 */

#ifndef UNBIASEDIMU_H_
#define UNBIASEDIMU_H_

#include "Sensors/UnbiasedSensor.h"
#include "Sensors/IMU.h"


class UnbiasedIMU : public UnbiasedSensor<IMUData> {
public:
	void filterData(IMUData* measurements, uint8_t count, IMUData* output);
};


#endif /* UNBIASEDIMU_H_ */
