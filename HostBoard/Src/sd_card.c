/*
 * sd_card.c
 *
 *  Created on: 15 Nov 2018
 *      Author: Clément Nussbaumer
 */

#include <stdbool.h>
#include "cmsis_os.h"
#include "fatfs.h"
#include "led.h"

#include <Misc/datastructs.h>
#include <Misc/Common.h>

#include <string.h>

#define MAX_FOLDER_NUMBER 1000
#define SD_BUFFER_SIZE 512
volatile bool new_sd_data_ready = 0;

FIL sensorsFile, eventsFile;
const TCHAR sensor_file_header[] = "CANSeq\tTimestamp[ms]\tID\tData\r\n";
const TCHAR events_file_header[] = "NOT USED timestamp, event_description\r\n";

volatile char sd_buffer1[SD_BUFFER_SIZE] = {0};
volatile char sd_buffer2[SD_BUFFER_SIZE] = {0};
volatile char *sd_buffer_tx = sd_buffer1;
volatile char *sd_buffer_rx = sd_buffer2;
volatile uint32_t sd_buffer_pointer_tx = 0;
volatile uint32_t sd_buffer_pointer_rx = 0;

int led_sdcard_id;

SemaphoreHandle_t buffer_semaphore = NULL;

void swap_buffer() {
	volatile uint32_t tmp_pointer;
	volatile char *tmp_buffer;

	if (xSemaphoreTake(buffer_semaphore, portMAX_DELAY) == pdTRUE) {
		tmp_buffer = sd_buffer_tx; // swap buffer
		sd_buffer_tx = sd_buffer_rx;
		sd_buffer_rx = tmp_buffer;

		tmp_pointer = sd_buffer_pointer_tx; // swap pointer
		sd_buffer_pointer_tx = sd_buffer_pointer_rx;
		sd_buffer_pointer_rx = tmp_pointer;

		xSemaphoreGive(buffer_semaphore);
	} else {
		// not reachable (Semaphore doesn't timeout)
	}
}

int sd_write(char str[], int size) {
	int ret = 0;

	if (xSemaphoreTake(buffer_semaphore, 0) == pdTRUE) {
		if (sd_buffer_pointer_tx+size < SD_BUFFER_SIZE) {
			for (int i=0 ; i<size ; i++) {
				sd_buffer_tx[sd_buffer_pointer_tx++] = str[i];
			}
			ret = size;
		} else {
			ret = 0;
		}
		xSemaphoreGive(buffer_semaphore);
	} else {
		ret = -1; // timed out in trying to obtain the semaphore
	}

	return ret;
}

osStatus initSdFile ()
{
  MX_FATFS_Init ();
  led_set_TK_rgb(led_sdcard_id, 0, 50, 50);

  buffer_semaphore = xSemaphoreCreateBinary(); // create buffer semaphore

  if (disk_initialize (0) != 0)
    {
      //The disk is not initialized correctly.
      return osErrorResource;
    }

  /*##-2- Register the file system object to the FatFs module ##############*/
  if (f_mount (&SDFatFS, (TCHAR const*) SDPath, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      return osErrorResource;
    }
  else
    {
      TCHAR dir[20];
      for (int i = 0; i < MAX_FOLDER_NUMBER; i++)
        {
          sprintf (dir, "DATA%04d", i);
          FILINFO info;
          if (f_stat (dir, &info) != FR_OK)
            {
              f_mkdir (dir);
              break;
            }
        }
      TCHAR path[100];

      sprintf (path, "%s" "/" "sensors.txt", dir);
      if (f_open (&sensorsFile, path, FA_OPEN_APPEND | FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
        {
          /* 'STM32.TXT' file Open for write Error */
          return osErrorResource;
        }
      sprintf (path, "%s" "/" "events.txt", dir);
      if (f_open (&eventsFile, path, FA_OPEN_APPEND | FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
        {
          /* 'STM32.TXT' file Open for write Error */
          return osErrorResource;
        }

      return osOK;
    }
}

FRESULT sync_sd_card(FRESULT result) {
	static uint32_t lastSync = 0;
	static int init = 0;

	if (!init) {
		lastSync = HAL_GetTick ();
		init = 1;
	}

	if ((HAL_GetTick () - lastSync) > 1000) // synchronization every second with the SD card.
	{
	  result |= f_sync (&sensorsFile);
	  lastSync = HAL_GetTick ();
	}
	return result;
}

void TK_sd_sync (void const* pvArgs)
{
  osDelay (200);
  led_sdcard_id = led_register_TK();

  if (initSdFile () != osOK)
    {
	  while (1) {
		  led_set_TK_rgb(led_sdcard_id, 50, 0, 0);
		  osDelay (1000);
	  }
    }

  UINT bytes_written = 0;
  f_write (&sensorsFile, sensor_file_header, strlen (sensor_file_header), &bytes_written);
  f_write (&eventsFile, events_file_header, strlen (events_file_header), &bytes_written);

  f_sync (&sensorsFile);
  f_sync (&eventsFile);

  osDelay (1000);
  FRESULT result = 0;

  xSemaphoreGive(buffer_semaphore);

  for (;;)
    {
      if (sd_buffer_pointer_rx == 0 && sd_buffer_pointer_tx > 0) {
    	  // nothing to write in rx buffer, but stuff awaiting in tx
    	  swap_buffer(); // will wait next tick to write
      }

      if (sd_buffer_pointer_rx > 0) {
    	  result |= f_write (&sensorsFile, sd_buffer_rx, sd_buffer_pointer_rx, &bytes_written);
    	  sd_buffer_pointer_rx -= bytes_written;
      }

      if (sd_buffer_pointer_rx == 0 && sd_buffer_pointer_tx > 0) {
    	  // nothing to write in rx buffer, but stuff awaiting in tx
    	  swap_buffer(); // will wait next tick to write
      }

      result = sync_sd_card(result);

      if (result == FR_OK) {
    	  led_set_TK_rgb(led_sdcard_id, 0, 50, 0);
      } else {
    	  led_set_TK_rgb(led_sdcard_id, 255, 0 ,0);

    	  osDelay(100);
    	  FATFS_DeInit();
    	  result = initSdFile();
    	  if (result == osOK) {
    		  f_write (&sensorsFile, sensor_file_header, strlen (sensor_file_header), &bytes_written);
    		  f_write (&eventsFile, events_file_header, strlen (events_file_header), &bytes_written);
    	  }
      }
      osDelay(10);
    }
}
