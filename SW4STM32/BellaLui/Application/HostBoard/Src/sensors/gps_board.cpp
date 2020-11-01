/*
 *
 * GPS IS DEPRECATED SINCE 2020 DESIGN AND HAS BEEN DISABLED SINCE 2021 DESIGN
 * THIS FILE
 *
 */


//#include "sensors/GPS/TinyGPS++.h"

#include "debug/led.h"
#include "debug/console.h"

extern "C" {
	#include "debug/profiler.h"
	#include "debug/monitor.h"
}

#include "can_transmission.h"

#include "storage/flash_logging.h"

#include <stm32f4xx_hal.h>
#include <cmsis_os.h>
#include <sensors/gps_board.h>
#include <usart.h>

#include <stdint.h>

#define GPS_RX_BUFFER_SIZE 256


UART_HandleTypeDef* gps_huart;

volatile uint32_t lastGpsDmaStreamIndex = 0;
volatile uint32_t endGpsDmaStreamIndex = 0;

uint8_t gpsRxBuffer[GPS_RX_BUFFER_SIZE];

// TinyGPSPlus gpsParser;


void gps_init(UART_HandleTypeDef* gpsHuart) {
	gps_huart = gpsHuart;
}

/*
 * Returns 0 if the GPS data was successfully transmitted
 * Returns 1 if the GPS is connected to satellites but could not determine any location
 * Returns 2 if the GPS is not connected to any satellite
 */
/*int8_t send_gps_data() {
	uint8_t sats = gpsParser.satellites.isValid() ? static_cast<uint8_t>(gpsParser.satellites.value()) : 0;

	if(enter_monitor(GPS_MONITOR)) {
		rocket_log(" Available satellites: %d\x1b[K\n", sats);
		rocket_log(" Longitude: %d [µdeg]\x1b[K\n", (uint32_t) (1000000 * gpsParser.location.lng()));
		rocket_log(" Latitude: %d [µdeg]\x1b[K\n", (uint32_t) (1000000 * gpsParser.location.lat()));
		rocket_log(" Altitude: %d [m]\x1b[K\n", (uint32_t) (gpsParser.altitude.value()));
		rocket_log(" HDOP: %d%%\x1b[K\n", (uint32_t) (100 * gpsParser.hdop.hdop()));

		exit_monitor(GPS_MONITOR);
	}

	can_setFrame((int32_t) sats, DATA_ID_GPS_SATS, HAL_GetTick());

	if(gpsParser.location.isValid()) {
		float hdop = gpsParser.hdop.isValid() ? gpsParser.hdop.hdop() : 0xffffffff;
		float lat = gpsParser.location.isValid() ? gpsParser.location.lat() : 0xffffffff;
		float lon = gpsParser.location.isValid() ? gpsParser.location.lng() : 0xffffffff;
		int32_t altitude = gpsParser.altitude.isValid() ? gpsParser.altitude.value() : 0;

		can_setFrame((int32_t) ((1E3) * hdop), DATA_ID_GPS_HDOP, HAL_GetTick());
		can_setFrame((int32_t) ((1E6) * lat), DATA_ID_GPS_LAT, HAL_GetTick());
		can_setFrame((int32_t) ((1E6) * lon), DATA_ID_GPS_LONG, HAL_GetTick());
		can_setFrame((int32_t) altitude, DATA_ID_GPS_ALTITUDE, HAL_GetTick());

		return 0;
	} else if(sats > 0){
		return 1;
	} else {
		return 2;
	}
}*/

void TK_GPS_board(void const* argument) {
	/*uint8_t gps_led = led_register_TK();

	uint32_t last_transmission = HAL_GetTick();

	HAL_UART_Receive_DMA(gps_huart, gpsRxBuffer, GPS_RX_BUFFER_SIZE);

	while(true) {
		start_profiler(1);

		endGpsDmaStreamIndex = GPS_RX_BUFFER_SIZE - gps_huart->hdmarx->Instance->NDTR;

		while(lastGpsDmaStreamIndex < endGpsDmaStreamIndex) {
			// rocket_log("%c", gpsRxBuffer[lastGpsDmaStreamIndex]);
			gpsParser.encode(gpsRxBuffer[lastGpsDmaStreamIndex++]);
		}

		if(((HAL_GetTick() - last_transmission) > 100) && gpsParser.passedChecksum() > 1) {
			switch(send_gps_data()) {
			case 0:
				led_set_TK_rgb(gps_led, 0x00, 0xFF, 0x00);
				break;
			case 1:
				led_set_TK_rgb(gps_led, 0x00, 0x00, 0xFF);
				break;
			default:
				led_set_TK_rgb(gps_led, 0xFF, 0x00, 0x00);
				break;
			}

			last_transmission = HAL_GetTick();
		}

		end_profiler();

		osDelay(10);
	}*/
}

void GPS_RxCpltCallback() {
	/*while(lastGpsDmaStreamIndex < GPS_RX_BUFFER_SIZE) {
		gpsParser.encode(gpsRxBuffer[lastGpsDmaStreamIndex++]);
	}

	endGpsDmaStreamIndex = 0;
	lastGpsDmaStreamIndex = 0;*/
}
