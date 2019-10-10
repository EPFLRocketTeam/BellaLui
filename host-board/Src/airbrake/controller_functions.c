/*
 * controller_functions.c
 *
 *  Created on: 28 avr. 2019
 *      Author: Alexandre
 */

#include <stm32f4xx_hal.h>
#include <string.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "led.h"

#include "lookup_table_shuriken.h"



#define MAX_OPENING_DEG 210 // deg
#define MIN_OPENING_DEG 0
#define ANGLE_HELLOWORLD 2

#define AB_RX_BUFFER_SIZE 64
#define FRAME_SIZE

UART_HandleTypeDef* airbrake_huart;
char abRxBuffer[AB_RX_BUFFER_SIZE] = {0};
volatile uint32_t lastABDmaStreamIndex = 0, endABDmaStreamIndex = 0;
volatile bool feedback_received = false;

volatile int char_processed = 0;

char command_string[10] = {0};

void ab_init(UART_HandleTypeDef *ab_huart) {
	airbrake_huart = ab_huart;
}

void ab_rx_parse(char c) {
	static char frame_buf[FRAME_SIZE] = {0};
	static int frame_idx = 0;
	static bool valid_frame = false;
	char_processed++;

	if (!feedback_received) { // still no response from AB
		if (c=='\r' || c=='\n' ) {
			valid_frame = true;
			frame_idx = 0;
		} else if (!valid_frame) {
			// waiting for footer
			// do nothing
		} else { // valid frame, just process the character
			frame_buf[frame_idx++] = c;

			if (frame_idx==1) {
				if (frame_buf[0] == 'O') { // recieved the first O of OK
					feedback_received = true;
				}
			} else {
				valid_frame = false;
			}
		}
	} else { // already got a positive feedback
		// do nothing
	}
}

void AB_RxCpltCallback ()
{
  while (lastABDmaStreamIndex < AB_RX_BUFFER_SIZE)
    {
	  ab_rx_parse(abRxBuffer[lastABDmaStreamIndex++]);
    }

  endABDmaStreamIndex = 0;
  lastABDmaStreamIndex = 0;
}

UART_HandleTypeDef* ab_gethuart() {
	return airbrake_huart;
}

void transmit_command(char* command, int size)
{
  static char command_buffer[64];
  strcpy(command_buffer, command);
  HAL_UART_Transmit(airbrake_huart, (uint8_t*)command_buffer, size, 100);
}


int deg2inc(float degrees_angle)
{
  //int inc = -(int) (degrees_angle * 75000 / 360); //3000 inc/evolution, 1:25reductor
	int inc = -(int) ((degrees_angle * 11806) / 360); //3000 inc/evolution, 1:12.76reductor
  return inc;
}

char* do_string_command (char first, char second, int number)
{
  sprintf(command_string, "%c%c%d\n", first, second, number);
  return command_string; //is it really needed ?
}

void motor_goto_position_inc (int position_inc)
{
  char command[15];
  transmit_command("EN\n", 3);
  do_string_command ('L', 'A', position_inc);
  sprintf(command, "%s%s", command_string, "M\n");
  transmit_command(command, strlen(command));

  can_setFrame(position_inc, DATA_ID_AB_INC, HAL_GetTick());
  can_setFrame(feedback_received, DATA_ID_AB_STATE, HAL_GetTick());
}

void controller_test (void)
{
	transmit_command("HO\r", 3);
    transmit_command("EN\r", 3);
    motor_goto_position_inc(10000);
    transmit_command("M\r", 2);
    return; // SMALL FUNCTION TO TEST IF THE TRANSMISSION IS WORKING... THE MOTOR SHOULD ROTATE
}


int aerobrakes_control_init (void)
{
	char command[64];
	char buffer[64] = {0};

	do_string_command ('L', 'L', deg2inc (MAX_OPENING_DEG));
	sprintf(command, "%s%s%s%s", "HO\n", "LL1\n", command_string, "APL1\n");
	transmit_command(command, strlen(command));

	// controller properties
	sprintf(command, "%s%s%s%s%s%s%s", "POR10\n", "I50\n", "PP30\n", "PD3\n",
			"LPC8000\n", "LCC2250\n", "EN\n");
	transmit_command(command, 37);

	// check rx buffer content
	while (HAL_UART_Receive(airbrake_huart, buffer, 1, 100) == HAL_OK) {
		ab_rx_parse (buffer[0]);
	}

	osDelay(100);

	can_setFrame(feedback_received, DATA_ID_AB_STATE, HAL_GetTick());

	return feedback_received;
}

void full_open (void)
{
  int angle_open_inc = deg2inc (MAX_OPENING_DEG);
  motor_goto_position_inc (angle_open_inc);

  return;
}

void full_close (void)
{
  int angle_close_inc = deg2inc (MIN_OPENING_DEG);
  motor_goto_position_inc (angle_close_inc);
  return;
}

void aerobrake_helloworld (void)
{
	for(int i=0; i<5;i++)
	{
		int angle_helloworld_inc = deg2inc(ANGLE_HELLOWORLD);
		motor_goto_position_inc(angle_helloworld_inc);
		osDelay(500);
		full_close();
		osDelay(300);
	}
}


float angle_tab (float altitude, float speed)
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

void command_aerobrake_controller (float altitude, float speed)
{
  float opt_act_position_deg = angle_tab (altitude, speed);

  int command_inc = deg2inc (opt_act_position_deg);
  motor_goto_position_inc(command_inc);
  return;
}
