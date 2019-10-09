#include "Sensors/GPS_board.h"
#include "Sensors/GPS/TinyGPS++.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "led.h"
#include "CAN_communication.h"
#include "usart.h"
#include <inttypes.h>

UART_HandleTypeDef* gps_huart;
int led_gps_id;

#define GPS_RX_BUFFER_SIZE 256
volatile uint32_t lastGpsDmaStreamIndex = 0, endGpsDmaStreamIndex = 0;
uint8_t gpsRxBuffer[GPS_RX_BUFFER_SIZE];

TinyGPSPlus gpsParser;


void gps_init(UART_HandleTypeDef *gpsHuart) {
	gps_huart = gpsHuart;
	led_gps_id = led_register_TK();
}

void send_gps_data() {
	uint8_t sats   = gpsParser.satellites.isValid () ? static_cast<uint8_t> (gpsParser.satellites.value ()) : 0;

	if (gpsParser.location.isValid ()) {
		float hdop = gpsParser.hdop.isValid () ? gpsParser.hdop.hdop () : 0xffffffff;
		float lat  = gpsParser.location.isValid () ? gpsParser.location.lat () : 0xffffffff;
		float lon  = gpsParser.location.isValid () ? gpsParser.location.lng () : 0xffffffff;
		int32_t altitude = gpsParser.altitude.isValid () ? gpsParser.altitude.value () : 0;

		can_setFrame((int32_t)((1e3)*hdop), DATA_ID_GPS_HDOP, HAL_GetTick());
		can_setFrame((int32_t)((1e6)*lat), DATA_ID_GPS_LAT, HAL_GetTick());
		can_setFrame((int32_t)((1e6)*lon), DATA_ID_GPS_LONG, HAL_GetTick());
		can_setFrame((int32_t)altitude, DATA_ID_GPS_ALTITUDE, HAL_GetTick());
		led_set_TK_rgb(led_gps_id, 0, 150, 0);
	} else if( gpsParser.satellites.isValid ()){
		led_set_TK_rgb(led_gps_id, 0, 0, 150);
	}
	can_setFrame((int32_t)sats, DATA_ID_GPS_SATS, HAL_GetTick());
}

void TK_GPS_board(void const * argument)
{
	  uint32_t measurement_time = HAL_GetTick ();
	  led_set_TK_rgb(led_gps_id, 50, 0, 150);
	  HAL_UART_Receive_DMA (gps_huart, gpsRxBuffer, GPS_RX_BUFFER_SIZE);


	  for (;;)
	    {
	      endGpsDmaStreamIndex = GPS_RX_BUFFER_SIZE - gps_huart->hdmarx->Instance->NDTR;
	      while (lastGpsDmaStreamIndex < endGpsDmaStreamIndex)
	        {
	          gpsParser.encode (gpsRxBuffer[lastGpsDmaStreamIndex++]);
	        }

	      if (((HAL_GetTick () - measurement_time) > 100) && // periodically
	    		  gpsParser.passedChecksum() > 1) // if a valid packet has been received
	        {
	    	  send_gps_data();
	          measurement_time = HAL_GetTick ();
	        }
	      osDelay (10);
	    }
}

void GPS_RxCpltCallback ()
{
  while (lastGpsDmaStreamIndex < GPS_RX_BUFFER_SIZE)
    {
      gpsParser.encode (gpsRxBuffer[lastGpsDmaStreamIndex++]);
    }

  endGpsDmaStreamIndex = 0;
  lastGpsDmaStreamIndex = 0;
}

UART_HandleTypeDef* gps_gethuart() {
	return gps_huart;
}
