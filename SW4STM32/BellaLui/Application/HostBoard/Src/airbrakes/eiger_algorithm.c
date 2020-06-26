/*
 * eiger_algorithm.c
 *
 *  Created on: 26 Jun 2020
 *      Author: Arion
 */


#include <airbrakes/eiger_algorithm.h>
#include <airbrakes/lookup_table_shuriken.h>



#define MAX_OPENING_DEG 210 // deg
#define MIN_OPENING_DEG 0



float eiger_angle_tab(float altitude, float speed)
{
  int index_altitude = 0;
  if (altitude < SimData[0][0])
  {
    return 0.0;
  }
  else if (altitude > SimData[TABLE_LENGTH - 1][0])
  {
    return (float) MAX_OPENING_DEG;
    }
  else
  {
    int j;
    float mean_speed_vector[TABLE_DIFF_SPEEDS_SAME_ALTITUDE];
    float mean_angle_vector[TABLE_DIFF_SPEEDS_SAME_ALTITUDE];
    while (SimData[index_altitude][0] < altitude)
    {
      index_altitude += TABLE_DIFF_SPEEDS_SAME_ALTITUDE;
    }
    float phi = (altitude - SimData[index_altitude - TABLE_DIFF_SPEEDS_SAME_ALTITUDE][0])
          / (SimData[index_altitude][0] - SimData[index_altitude - TABLE_DIFF_SPEEDS_SAME_ALTITUDE][0]);
    for (j = 0; j < TABLE_DIFF_SPEEDS_SAME_ALTITUDE; j++)
    {
      mean_speed_vector[j] = (1-phi) * SimData[index_altitude - TABLE_DIFF_SPEEDS_SAME_ALTITUDE + j][1]
              + (phi) * SimData[index_altitude + j][1];
      mean_angle_vector[j] = (1-phi) * SimData[index_altitude - TABLE_DIFF_SPEEDS_SAME_ALTITUDE + j][2]
              + (phi) * SimData[index_altitude + j][2];
    }

    int index_speed = 0;
    if (speed < mean_speed_vector[0])
    {
      return 0.0;
    }
    else if (speed > mean_speed_vector[TABLE_DIFF_SPEEDS_SAME_ALTITUDE - 1])
    {
      return (float) MAX_OPENING_DEG;
    }
    else
    {
      while (mean_speed_vector[index_speed] < speed)
      {
        index_speed += 1;
      }
      float theta = (speed - mean_speed_vector[index_speed - 1])
              / (mean_speed_vector[index_speed] - mean_speed_vector[index_speed - 1]);
      float mean_angle = (1-theta) * mean_angle_vector[index_speed - 1] + (theta) * mean_angle_vector[index_speed];
      return mean_angle;
    }
  }
}
