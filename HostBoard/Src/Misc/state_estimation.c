/*
 * state_estimation.c
 *
 *  Created on: 18 Jun 2018
 *      Author: Cl�ment Nussbaumer
 */

#include <Misc/Common.h>
#include <Misc/rocket_constants.h>
#include <cmsis_os.h>
#include "CAN_communication.h"

volatile float32_t air_speed_state_estimate, altitude_estimate;
#define ALTITUDE_BUFFER_SIZE 8

void TK_state_estimation ()
{
  float32_t altitude_buffer[ALTITUDE_BUFFER_SIZE][2];
  uint8_t altitude_index = 0;
  BARO_data baro;

  uint32_t lastBaroSeqNumber = 0;

  while (LIFTOFF_TIME == 0)
    {
      if (lastBaroSeqNumber < currentBaroSeqNumber)
        {
          altitude_index++;
          baro = BARO_buffer[currentBaroSeqNumber % CIRC_BUFFER_SIZE];
          altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][0] = baro.altitude - baro.base_altitude;
          altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][1] = currentBaroTimestamp;
          lastBaroSeqNumber = currentBaroSeqNumber;
        }

      osDelay (10);
    }

  for (;;)
    {

      if (lastBaroSeqNumber < currentBaroSeqNumber)
        {
          altitude_index++;
          BARO_data baro = BARO_buffer[currentBaroSeqNumber % CIRC_BUFFER_SIZE];
          altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][0] = baro.altitude - baro.base_altitude;
          altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][1] = currentBaroTimestamp;

          lastBaroSeqNumber = currentBaroSeqNumber;
        }

      float32_t d_t = altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][1]
          - altitude_buffer[(altitude_index + 1) % ALTITUDE_BUFFER_SIZE][1];
      d_t /= 1000.0;

      float32_t d_h = altitude_buffer[altitude_index % ALTITUDE_BUFFER_SIZE][0]
          - altitude_buffer[(altitude_index + 1) % ALTITUDE_BUFFER_SIZE][0];

      float32_t sum = 0.0f;
      for (int i = 0; i < ALTITUDE_BUFFER_SIZE; ++i)
        {
          sum += altitude_buffer[i][0];
        }

      altitude_estimate = sum / ALTITUDE_BUFFER_SIZE;
      air_speed_state_estimate = d_h / d_t;

      //can_setFrame((int32_t) altitude_estimate, DATA_ID_AB_ALT, HAL_GetTick());
      //can_setFrame((int32_t) (air_speed_state_estimate*1000), DATA_ID_AB_AIRSPEED, HAL_GetTick());

      osDelay (3);
    }

}
