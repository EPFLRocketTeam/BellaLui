/*
 * state_machine.c
 *
 *  Created on: 24 Apr 2018
 *      Author: ebrunner
 */

#include <cmsis_os.h>
#include <stm32f4xx_hal.h>

#include <Misc/Common.h>
#include <Misc/rocket_constants.h>
#include "CAN_communication.h"

void TK_state_machine (void const * argument)
{

  osDelay (2000);

  // Declare time variable
  uint32_t time_tmp = 0;

  // Declare sensor variables
  IMU_data* imu_data;
  BARO_data* baro_data;
  uint32_t lastImuSeqNumber = 0, lastBaroSeqNumber = 0;
  uint8_t imuIsReady = 0, baroIsReady = 0;

  // Declare apogee detection variables
  float32_t max_altitude = 0;
  uint32_t apogee_counter = 0;

  // Declare secondary recovery event detection variables
  uint32_t sec_counter = 0;

  // Declare touch-down event detection variables
  float32_t td_last_alt = 0;
  uint32_t td_counter = 0;

  // TODO: Set low package data rate

  // State Machine initialization
  // Hyp: rocket is on rail waiting for lift-off
  currentState = STATE_CALIBRATION;

  // State Machine main task loop
  for (;;)
    {
      osDelay (10);
      can_setFrame(currentState, DATA_ID_STATE, HAL_GetTick());

      // if new imu data is available
      if (currentImuSeqNumber > lastImuSeqNumber)
        {
          // Update accelerometer reading
          imu_data = getCurrentIMU_data ();
          lastImuSeqNumber = currentImuSeqNumber;
          imuIsReady = 1; // set new data flag to true
        }
      else
        {
          imuIsReady = 0; // set new data flag to false
        }

      // if new barometer data is available
      if (currentBaroSeqNumber > lastBaroSeqNumber)
        {
          // Update barometer reading
          baro_data = getCurrentBARO_data ();
          lastBaroSeqNumber = currentBaroSeqNumber;
          baroIsReady = 1; // set new data flag to true
        }
      else
        {
          baroIsReady = 0; // set new data flag to false
        }

      if (LIFTOFF_TIME != 0 && (HAL_GetTick() - LIFTOFF_TIME) > 4 * 60 * 1000) {
          currentState = STATE_TOUCHDOWN;
      }

      // State Machine
      switch (currentState)
        {
        case STATE_CALIBRATION:
          {
            if (baroIsReady)
              {
                if (baro_data->base_pressure != 0)
                  {
                    currentState = STATE_IDLE;
                  }
                currentState = STATE_IDLE;
              }
            break;
          }

        case STATE_IDLE:
          {
            if (imuIsReady)
              {

                // Compute lift-off triggers for acceleration
                uint8_t liftoffAccelTrig = (abs_fl32 (imu_data->acceleration.z) > ROCKET_CST_LIFTOFF_TRIG_ACCEL);

                if (LIFTOFF_TIME != 0)
                  {
                    //already detected the acceleration trigger. now we need the trigger for at least 1000ms before trigerring the liftoff.
                    if (liftoffAccelTrig && HAL_GetTick () - LIFTOFF_TIME > LIFTOFF_DETECTION_DELAY)
                      {
                        currentState = STATE_LIFTOFF; // Switch to lift-off state
                        break;
                      }
                    else if (!liftoffAccelTrig) //false positive.
                      {
                        LIFTOFF_TIME = 0;
                        time_tmp = 0;
                      }
                    break;
                  }
                // detect lift-off
                if (liftoffAccelTrig)
                  {
                    LIFTOFF_TIME = HAL_GetTick ();
                    time_tmp = HAL_GetTick (); // Start timer to estimate motor burn out
                  }
              }
            break;
          }

        case STATE_LIFTOFF:
          {
            flight_status = 10;
            uint32_t currentTime = HAL_GetTick ();
            // determine motor burn-out based on lift-off detection
            if ((currentTime - time_tmp) > ROCKET_CST_MOTOR_BURNTIME)
              {
                currentState = STATE_COAST; // switch to coast state
              }
            break;
          }

        case STATE_COAST:
          {
            flight_status = 20;

            // compute apogee triggers for altitude
            if (baroIsReady)
              {
                uint8_t minAltTrig = ((baro_data->altitude - baro_data->base_altitude) > ROCKET_CST_MIN_TRIG_AGL);
                uint8_t counterAltTrig = 0;
                uint8_t diffAltTrig = 0;

                // update the maximum altitude detected up to this point
                if (max_altitude < baro_data->altitude)
                  {
                    // if a new maximum altitude is found, this means the rocket is going up
                    max_altitude = baro_data->altitude;
                    // The descending measurement counter is thus set to zero
                    apogee_counter = 0;
                  }
                else
                  {
                    // if the rocket isn't rising then it is descending, thus the number of descending measurements are counted
                    apogee_counter++;
                    if (apogee_counter > APOGEE_BUFFER_SIZE)
                    // if the number of measurements exceeds a certain value (basic noise filtering)...
                      {
                        // ... then the altitude trigger based on the counter is enabled
                        counterAltTrig = 1;
                        if (( max_altitude - baro_data->altitude) > APOGEE_ALT_DIFF)
                        // since the rocket is then supposed to be descending, the trigger waits for an altitude offset greater than the one defined to occure before triggering the state change
                          {
                            diffAltTrig = 1;
                          }
                      }
                  }

                // detect apogee
                if (minAltTrig && counterAltTrig && diffAltTrig)
                  {
                    time_tmp = HAL_GetTick (); // save time to mute sensors while ejection occures
                    currentState = STATE_PRIMARY; // switch to primary descent phase
                    flight_status = 30;
                  }
              }
            break;
          }

        case STATE_PRIMARY:
          {
            if (baroIsReady)
              {
                // check that some time has passed since the detection of the apogee before triggering the secondary recovery event
                uint8_t sensorMuteTimeTrig = ((HAL_GetTick () - time_tmp) > APOGEE_MUTE_TIME);
                uint8_t counterSecTrig = 0;

                // update the minimum altitude detected up to this point
                if ((baro_data->altitude - baro_data->base_altitude) > ROCKET_CST_REC_SECONDARY_ALT)
                  {
                    // As long as the measured altitude is above the secondary recovery event altitude, keep buffer counter to 0
                    sec_counter = 0;
                  }
                else
                  {
                    // if the measured altitude is lower than the trigger altitude, start counting
                    sec_counter++;
                    if (sec_counter > SECONDARY_BUFFER_SIZE)
                    // if more than a given amount of measurements are below the secondary recovery altitude, toggle the state trigger
                      {
                        counterSecTrig = 1;
                      }
                  }

                //detect secondary recovery event
                if (sensorMuteTimeTrig && counterSecTrig)
                  {
                    time_tmp = HAL_GetTick (); // save current time to start differed touchdown detection rate
                    currentState = STATE_SECONDARY; // switch to secondary recovery phase
                    td_last_alt = baro_data->altitude; // save altitude measurement for touchdown detection
                    flight_status = 35;

                  }
              }
            break;
          }

        case STATE_SECONDARY:
          {
            uint8_t counterTdTrig = 0;

            // if a given time has passed since the last time the check was done, do the check
            if ((HAL_GetTick () - time_tmp) > TOUCHDOWN_DELAY_TIME)
              {
                if (baroIsReady)
                  {
                    // save time of check
                    time_tmp = HAL_GetTick ();
                    // check if the altitude hasn't varied of more than a given amount since last time
                    if (abs_fl32 (baro_data->altitude - td_last_alt) > TOUCHDOWN_ALT_DIFF)
                      {
                        // if the altitude difference is still large, this means the rocket is descending and so the touch down counter is kept to zero
                        td_counter = 0;
                      }
                    else
                      {
                        // if the altitude difference is in bounds, the counter is incremented
                        td_counter++;
                        if (td_counter > TOUCHDOWN_BUFFER_SIZE)
                          {
                            // if the counter is larger than a given value, toggle the state trigger
                            counterTdTrig = 1;
                          }
                      }
                    td_last_alt = baro_data->altitude;

                    if (counterTdTrig)
                      {
                        currentState = STATE_TOUCHDOWN;
                        flight_status = 40;
                        // TODO: Set telemetry data rate to low
                      }
                  }
              }

            break;
          }

        case STATE_TOUCHDOWN:
        	osDelay(2000);
            break;
        }

    }
}
