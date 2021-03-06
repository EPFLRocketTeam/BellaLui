/*
 * controller_functions.c
 *
 *  Created on: 28 avr. 2019
 *      Author: Alexandre
 */

#include <stm32f4xx_hal.h>
#include <cmsis_os.h>

#include <string.h>
#include <stdbool.h>

#include "airbrakes/ab_algorithm.h"
#include "debug/monitor.h"
#include "can_transmission.h"

#define SIMULATE_AIRBRAKES

#define MAX_OPENING_DEG 210 // deg
#define MIN_OPENING_DEG 0
#define ANGLE_HELLOWORLD 5

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
#ifndef SIMULATE_AIRBRAKES
  char command[15];
  transmit_command("EN\n", 3);
  do_string_command ('L', 'A', position_inc);
  sprintf(command, "%s%s", command_string, "M\n");
  transmit_command(command, strlen(command));
#endif

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
#ifndef SIMULATE_AIRBRAKES
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
#else
	return true;
#endif

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


void command_aerobrake_controller (float altitude, float speed)
{
  float opt_act_position_deg = angle_tab(altitude, speed);

  if((int32_t) opt_act_position_deg != -190) {
#ifdef SIMULATE_AIRBRAKES
	  rocket_log("Airbrakes angle is %d° at %dms\x1b[K\n", (int32_t) opt_act_position_deg, HAL_GetTick());
#endif

	 int command_inc = deg2inc(opt_act_position_deg);
	 motor_goto_position_inc(command_inc);
  }
}

float angle_final;

void test_ab() {
	for(int i = 1000; i < 2000; i++) {
		for(int j = 0; j < 500; j++) {
			float angle = angle_tab((float) i, (float) j);

			if((int) angle != -190) {
				command_aerobrake_controller((float) i, (float) j);

				for(int k = 0; k < 1; k++) {
					angle += k;
				}

				angle_final = angle;

				break;
			}
		}
	}
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

	// test_ab();
}
