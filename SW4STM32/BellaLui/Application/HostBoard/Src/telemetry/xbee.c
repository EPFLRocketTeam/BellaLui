/*
 * xbee.c
 *
 *  Created on: 9 Apr 2018
 *      Author: Clement Nussbaumer
 */

#include <debug/led.h>
#include <misc/datastructs.h>
#include <stddef.h>
#include <stm32f4xx_hal_uart.h>
#include <sys/_stdint.h>
#include <telemetry/telemetry_handling.h>
#include <telemetry/telemetry_protocol.h>
#include <telemetry/xbee.h>

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
#define XBEE_CHECKSUM_SIZE 1 // checksum size of the XBee packet

#include "stm32f4xx_hal.h"
#include "Misc/Common.h"

// XBee receiving mode
#define XBEE_RECEIVED_DATAGRAM_ID_INDEX 16
#define XBEE_RX_BUFFER_SIZE 512
#define RX_PACKET_SIZE 64
#define XBEE_RECEIVED_DATA_LENGTH_INDEX 2

uint8_t packetSize = 64; // Size of the datagram received
uint8_t rxPacketBuffer[RX_PACKET_SIZE];  // Buffer with one datagram to process only
uint32_t lastDmaStreamIndex = 0, endDmaStreamIndex = 0;
uint8_t rxBuffer[XBEE_RX_BUFFER_SIZE];  // Buffer with all data received
uint32_t preambleCnt, packetCnt, currentChecksum;


enum DECODING_STATE
{
  PARSING_PACKET, PARSING_CHECKSUM
};

uint8_t currentRxState = PARSING_PACKET;

static uint8_t XBEE_FRAME_OPTIONS[XBEE_OPTIONS_SIZE] =
  {
  XBEE_TX_FRAME_TYPE,  // Frame type
      0x00,           // Frame ID
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,     // 64 bit dest address
      0xff, 0xfe,           // 16 bits dest address (0xff fe = broadcast)
      0x00,           // Broadcast radius (0 = max)
      0x43 };          // Transmit options (disable ACK and Route discovery)

uint32_t XBEE_FRAME_OPTIONS_CRC = 0;
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

void TK_xBeeTransmit (const void* args)
{
	/*while(true) {
		uint8_t command[] = {0x7E,
			                     0x00, 0x10, // length
			                     0x10,  // Frame type // Transmit Request frame - 0x10
			                     0x00,           // Frame ID - Setting it to '0' will disable response frame.
			                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,     // 64 bit dest address // broadcast
			                     0xff, 0xfe,           // 16 bits dest address (0xff fe = broadcast) unknown address
			                     0x00,           // Broadcast radius (0 = max) no hops
			                     0x43,
			                     0xff, //DATA
			                     0xfe, //DATA
			                     0xb4 // CRC
		};
		led_set_rgb(0,0,255);
		HAL_UART_Transmit_DMA (xBee_huart, command, sizeof(command));
		osDelay(1000);
		led_set_rgb(255,0,0);
		osDelay(1000);
	}*/




  led_xbee_id = led_register_TK();
  led_set_TK_rgb(led_xbee_id, 100, 50, 0);

  initXbee ();
  uint32_t packetStartTime = HAL_GetTick();



  for (;;)
    {

      uint32_t elapsed = HAL_GetTick() - packetStartTime;
      if ((currentXbeeTxBufPos > 0) && (elapsed) > XBEE_SEND_FRAME_TIMEOUT_MS)
        {
    	  //timeout reached and buffer not empty, sending frame whatever the content
          sendXbeeFrame();
          packetStartTime = HAL_GetTick();
        } else if (currentXbeeTxBufPos==0 && elapsed > XBEE_SEND_FRAME_LONG_TIMEOUT_MS) {
        	// force dummy frame creation
        	telemetry_sendIMUData((IMU_data) {{666.0f, 666.0f, 666.0f}, {666, 666, 666}, 666.0f});
        }
      osEvent event;
      do {
    	  //as long as there is data to send, send it.
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
  // send the XBee frame if there remains less than 20 bytes available in the txDataBuffer
  if (XBEE_PAYLOAD_MAX_SIZE - currentXbeeTxBufPos < 20)
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

/**
 * Sends the data contained in the buffer (the frame)
*/
void sendXbeeFrame ()
{
  if (osSemaphoreWait (xBeeTxBufferSemHandle, XBEE_UART_TIMEOUT) != osOK)
    {
	  //could not obtain free semaphore in given timeout delay, setting LED red
	  led_set_TK_rgb(led_xbee_id, 50, 0, 0);
      return;
    }

  uint16_t payloadAndConfigSize = XBEE_OPTIONS_SIZE + currentXbeeTxBufPos;

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
  //send the data buffer to the xBee module
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
/**
 * Initialises the Xbee module given parameters in the XBEE_FRAME_OPTIONS
 * Changes control register XBEE_FRAME_OPTIONS_CRC accordingly
 */
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

void TK_xBeeReceive (const void* args)
{
	HAL_UART_Receive_DMA (xBee_huart, rxBuffer, XBEE_RX_BUFFER_SIZE);

	for (;;)
	{
		endDmaStreamIndex = XBEE_RX_BUFFER_SIZE - xBee_huart->hdmarx->Instance->NDTR;
		while (lastDmaStreamIndex < endDmaStreamIndex)
		{
			processReceivedByte (rxBuffer[lastDmaStreamIndex++]);
		}

	osDelay (10);
	}
}

void xBee_rxCpltCallback ()
{
  while (lastDmaStreamIndex < XBEE_RX_BUFFER_SIZE)
    {
      processReceivedByte (rxBuffer[lastDmaStreamIndex++]);
    }

  endDmaStreamIndex = 0;
  lastDmaStreamIndex = 0;
}

void resetStateMachine ()
{
  currentRxState = PARSING_PACKET;
  packetCnt = 0;
  currentChecksum = 0;
}

void processReceivedPacket ()
{
	switch (rxPacketBuffer[XBEE_RECEIVED_DATAGRAM_ID_INDEX])
	{
		case ORDER_PACKET:
		{
			uint8_t* RX_Order_Packet = rxPacketBuffer + START_DELIMITER_SIZE + MSB_SIZE + LSB_SIZE + XBEE_RECEIVED_OPTIONS_SIZE + DATAGRAM_ID_SIZE + PREFIXE_EPFL_SIZE;
			telemetry_receiveOrderPacket(RX_Order_Packet);
			break;
		}
		case IGNITION_PACKET:
		{
			uint8_t* RX_Ignition_Packet = rxPacketBuffer + START_DELIMITER_SIZE + MSB_SIZE + LSB_SIZE + XBEE_RECEIVED_OPTIONS_SIZE + DATAGRAM_ID_SIZE + PREFIXE_EPFL_SIZE;
			telemetry_receiveIgnitionPacket(RX_Ignition_Packet);
			break;
		}
		default :
		{
			break;
		}
	}
}

inline void processReceivedByte (uint8_t rxByte)
{
  switch (currentRxState)
    {
    case PARSING_PACKET:
      {
    	rxPacketBuffer[packetCnt++] = rxByte;
        if (packetCnt > 2) {
        	currentChecksum += rxByte;
        }
        if (packetCnt == XBEE_RECEIVED_DATAGRAM_ID_INDEX) {
        	set_packet_size(rxPacketBuffer[packetCnt]);
        }
        if ( packetCnt == (packetSize-CHECKSUM_SIZE) )
          {
            currentRxState = PARSING_CHECKSUM;
          }
        break;
      }
    case PARSING_CHECKSUM:
      {
        if (currentChecksum == rxByte)
          {
            processReceivedPacket ();
          }
        resetStateMachine ();
        break;
      }
    }
}

/* set_packet_size(uint8_t datagram_id)
 *
 * Sets the size of the data received depending on the datagram_id received
 *
 * The size is calculated as follow :
 * START_DELIMITER_SIZE + MSB_SIZE + LSB_SIZE + XBEE_RECEIVED_OPTIONS_SIZE + DATAGRAM_ID_SIZE + PREFIXE_EPFL_SIZE
 * + PREFIXE_EPFL_SIZE + TIMESTAMP_SIZE + PACKET_NUMBER_SIZE + XXX_DATAGRAM_PAYLOAD_SIZE
 *
 * Add the new packets size if news packets are needed
 */


void set_packet_size(uint8_t datagram_id) {
	switch (datagram_id)
	{
		case ORDER_PACKET:
	    {
	    	packetSize = ORDER_PACKET_SIZE;
	    	break;
	    }
		case IGNITION_PACKET:
		{
			packetSize = IGNITION_PACKET_SIZE;
	    	break;
		}
		case TELEMETRY_PACKET:
		{
			packetSize = TELEMETRY_PACKET_SIZE;
			break;
		}
		default :
		{
	    	break;
		}
	}
}


