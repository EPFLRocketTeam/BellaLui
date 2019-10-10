/*
 * data_handling.c
 *
 *  Created on: 19 Apr 2018
 *      Author: Clement Nussbaumer
 *      Alexandre Devienne
 */

#include "cmsis_os.h"

#include "Misc/Common.h"

#include "Misc/data_handling.h"
#include "Misc/datagram_builder.h"
#include "Telemetry/telemetry_protocol.h"
#include "Telemetry/simpleCRC.h"

#include <stdbool.h>

extern "C" {
#include "CAN_communication.h"
#include "sd_card.h"
}

#define TELE_TIMEMIN 100
#define GPS_TIMEMIN 500

// for import in C code
extern "C" bool telemetry_handleGPSData(GPS_data data);
extern "C" bool telemetry_handleIMUData(IMU_data data);
extern "C" bool telemetry_handleBaroData(BARO_data data);

extern osMessageQId xBeeQueueHandle;

uint32_t telemetrySeqNumber = 0;

IMU_data  imu  = {{0,0,0},{0,0,0}, 0};
BARO_data baro = {0,0,0};
uint32_t last_sensor_update = 0;
Telemetry_Message m1;
Telemetry_Message m2;
Telemetry_Message m3;

Telemetry_Message createTelemetryDatagram (IMU_data* imu_data, BARO_data* baro_data, uint32_t measurement_time, uint32_t telemetrySeqNumber)
{

  DatagramBuilder builder = DatagramBuilder (SENSOR_DATAGRAM_PAYLOAD_SIZE, TELEMETRY_ERT18, telemetrySeqNumber);

  // ## Beginning of datagram Payload ##
  // measurement time
  builder.write32<uint32_t> (measurement_time);

  builder.write32<float32_t> (imu_data->acceleration.x);
  builder.write32<float32_t> (imu_data->acceleration.y);
  builder.write32<float32_t> (imu_data->acceleration.z);

  builder.write32<float32_t> (can_getState()); // flight status
  builder.write32<float32_t> (can_getABangle()); // AB_angle
  builder.write32<float32_t> (can_getSpeed()); // pitot_press
  //baro_data->temperature = 20;
  builder.write32<float32_t> (baro_data->temperature);
  builder.write32<float32_t> (baro_data->pressure);

  builder.write32<float32_t> (can_getAltitude());

  return builder.finalizeDatagram ();
}


Telemetry_Message createGPSDatagram (uint32_t seqNumber, GPS_data gpsData)
{
  DatagramBuilder builder = DatagramBuilder (GPS_DATAGRAM_PAYLOAD_SIZE, GPS, seqNumber++);

  builder.write32<uint32_t> (HAL_GetTick ());
  builder.write8 (gpsData.sats);
  builder.write32<float32_t> (gpsData.hdop);
  builder.write32<float32_t> (gpsData.lat);
  builder.write32<float32_t> (gpsData.lon);
  builder.write32<int32_t> (gpsData.altitude);

  return builder.finalizeDatagram ();
}





bool telemetry_handleGPSData(GPS_data data) {
	static uint32_t last_update = 0;
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_update > GPS_TIMEMIN) {
		m1 = createGPSDatagram (telemetrySeqNumber++, data);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m1, 10) != osOK) {
			vPortFree(m1.ptr); // free the datagram if we couldn't queue it
		}
		last_update = now;
		handled = true;
	}
	return handled;
}


bool telemetry_handleIMUData(IMU_data data) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	imu = data;

	if (now - last_sensor_update > TELE_TIMEMIN) {
		m2 = createTelemetryDatagram (&imu, &baro, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m2, 10) != osOK) {
			vPortFree(m2.ptr); // free the datagram if we couldn't queue it
		}
		last_sensor_update = now;
		handled = true;
	}
	return handled;
}

bool telemetry_handleBaroData(BARO_data data) {
	uint32_t now = HAL_GetTick();
	bool handled = false;

	baro = data;

	if (now - last_sensor_update > TELE_TIMEMIN) {
		m3 = createTelemetryDatagram (&imu, &baro, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m3, 10) != osOK) {
			vPortFree(m3.ptr); // free the datagram if we couldn't queue it
		}
		last_sensor_update = now;
		handled = true;
	}
	return handled;
}
