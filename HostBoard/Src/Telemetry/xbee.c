/*
 * xbee.c
 *
 *  Created on: 9 Apr 2018
 *      Author: Clement Nussbaumer
 */

#include "stm32f4xx_hal.h"
#include "Misc/Common.h"
#include "FreeRTOS.h"
#include "led.h"
#include <Telemetry/xbee.h>
#include <Telemetry/telemetry_handling.h>

osMessageQId xBeeQueueHandle;
osSemaphoreId xBeeTxBufferSemHandle;

UART_HandleTypeDef* xBee_huart;

// UART settings
#define XBEE_UART_TIMEOUT 30
#define XBEE_SEND_FRAME_LONG_TIMEOUT_MS 1000

// XBee API mode
#define XBEE_START 0x7e
#define XBEE_ESCAPE 0x7d
#define XBEE_TX_FRAME_TYPE 0x10 // Transmit request frame
#define XBEE_FRAME_BEGINNING_SIZE 3 // Start delimiter (0x7E) + uint16_t length of the frame
#define XBEE_FRAME_OPTIONS_SIZE 14
#define XBEE_CHECKSUM_SIZE 1 // checksum size of the XBee packet

static uint8_t XBEE_FRAME_OPTIONS[XBEE_FRAME_OPTIONS_SIZE] =
  {
  XBEE_TX_FRAME_TYPE,  // Frame type
      0x00,           // Frame ID
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,     // 64 bit dest address
      0xff, 0xfe,           // 16 bits dest address (0xff fe = broadcast)
      0x00,           // Broadcast radius (0 = max)
      0x43 };          // Transmit options (disable ACK and Route discovery)

uint8_t XBEE_FRAME_OPTIONS_CRC = 0;
uint16_t XBEE_SEND_FRAME_TIMEOUT_MS = 32;

uint8_t currentCrc = 0;
uint8_t payloadBuffer[XBEE_PAYLOAD_MAX_SIZE];
uint8_t txDmaBuffer[2 * XBEE_PAYLOAD_MAX_SIZE + XBEE_CHECKSUM_SIZE + XBEE_FRAME_BEGINNING_SIZE];
uint16_t currentXbeeTxBufPos = 0;

int led_xbee_id;

void xbee_freertos_init(UART_HandleTypeDef *huart) {
	osSemaphoreDef(xBeeTxBufferSem);
	xBeeTxBufferSemHandle = osSemaphoreCreate(osSemaphore(xBeeTxBufferSem), 1);

	osMessageQDef(xBeeQueue, 16, Telemetry_Message);
	xBeeQueueHandle = osMessageCreate(osMessageQ(xBeeQueue), NULL);
	vQueueAddToRegistry (xBeeQueueHandle, "xBee incoming queue");

	xBee_huart = huart;
}

void TK_xBeeTelemetry (const void* args)
{
  led_xbee_id = led_register_TK();
  led_set_TK_rgb(led_xbee_id, 100, 50, 0);

  initXbee ();
  uint32_t packetStartTime = HAL_GetTick();

  for (;;)
    {

      uint32_t elapsed = HAL_GetTick() - packetStartTime;
      if ((currentXbeeTxBufPos > 0) && (elapsed) > XBEE_SEND_FRAME_TIMEOUT_MS)
        {
          sendXbeeFrame();
          packetStartTime = HAL_GetTick();
        } else if (currentXbeeTxBufPos==0 && elapsed > XBEE_SEND_FRAME_LONG_TIMEOUT_MS) {
        	// force dummy frame creation
        	telemetry_handleIMUData((IMU_data) {{1,1,1}, {0,0,0}, 0});
        }
      osEvent event;
      do {
       event = osMessageGet (xBeeQueueHandle, 5);
       if (event.status == osEventMessage)
       {
          if (currentXbeeTxBufPos == 0 && elapsed != 0){
              packetStartTime = HAL_GetTick();
          }

          Telemetry_Message* m = event.value.p;
          sendData (m->ptr, m->size);
          vPortFree (m->ptr);
        }
      } while(event.status == osEventMessage);

      osDelay(10);
    }
}

void receiveData ()
{
	// do nothing
}

void sendData (uint8_t* txData, uint16_t txDataSize)
{
  if (txDataSize >= XBEE_PAYLOAD_MAX_SIZE)
    {
      return;
    }

  if (currentXbeeTxBufPos + txDataSize >= XBEE_PAYLOAD_MAX_SIZE)
    {
      sendXbeeFrame ();
    }

  if (currentXbeeTxBufPos + txDataSize < XBEE_PAYLOAD_MAX_SIZE)
    {
      addToBuffer (txData, txDataSize);
    }

  if (XBEE_PAYLOAD_MAX_SIZE - currentXbeeTxBufPos < 20) // send the XBee frame if there remains less than 20 bytes available in the txDataBuffer
    {
      sendXbeeFrame ();
    }
}

inline void addToBuffer (uint8_t* txData, uint16_t txDataSize)
{
  for (uint16_t i = 0; i < txDataSize; i++)
    {
      payloadBuffer[currentXbeeTxBufPos + i] = txData[i];
    }
  currentXbeeTxBufPos += txDataSize;
}

void sendXbeeFrame ()
{
  if (osSemaphoreWait (xBeeTxBufferSemHandle, XBEE_UART_TIMEOUT) != osOK)
    {
	  led_set_TK_rgb(led_xbee_id, 50, 0, 0);
      return;
    }

  uint16_t payloadAndConfigSize = XBEE_FRAME_OPTIONS_SIZE + currentXbeeTxBufPos;

  uint16_t pos = 0;
  txDmaBuffer[pos++] = XBEE_START;
  txDmaBuffer[pos++] = payloadAndConfigSize >> 8;
  txDmaBuffer[pos++] = payloadAndConfigSize & 0xff;
  for (int i = 0; i < sizeof(XBEE_FRAME_OPTIONS); i++)
    {
      txDmaBuffer[pos++] = XBEE_FRAME_OPTIONS[i];
    }
  currentCrc = XBEE_FRAME_OPTIONS_CRC;

  for (int i = 0; i < currentXbeeTxBufPos; ++i)
    {
      uint8_t escapedChar;
      if ((escapedChar = escapedCharacter (payloadBuffer[i])))
        {
          txDmaBuffer[pos++] = XBEE_ESCAPE;
          txDmaBuffer[pos++] = escapedChar;
        }
      else
        {
          txDmaBuffer[pos++] = payloadBuffer[i];
        }

      currentCrc += payloadBuffer[i];
    }

  currentCrc = 0xff - currentCrc;
  txDmaBuffer[pos++] = currentCrc;
  HAL_UART_Transmit_DMA (xBee_huart, txDmaBuffer, pos);

  currentXbeeTxBufPos = 0;

  led_set_TK_rgb(led_xbee_id, 0, 50, 0);
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
	if (huart == xBee_huart) {
		osSemaphoreRelease (xBeeTxBufferSemHandle);
	}
}

void initXbee ()
{
  uint8_t checksum = 0;
  for (int i = 0; i < sizeof(XBEE_FRAME_OPTIONS); ++i)
    {
      checksum += XBEE_FRAME_OPTIONS[i];
    }
  XBEE_FRAME_OPTIONS_CRC = checksum;
}

inline uint8_t escapedCharacter (uint8_t byte)
{
  switch (byte)
    {
    case 0x7e:
      return 0x5e;
    case 0x7d:
      return 0x5d;
    case 0x11:
      return 0x31;
    case 0x13:
      return 0x33;
    default:
      return 0x00;
    }
}
