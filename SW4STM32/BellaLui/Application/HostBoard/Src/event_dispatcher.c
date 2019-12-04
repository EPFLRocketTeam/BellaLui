/*
 * event_dispatcher.c
 *
 *  Created on: 28 Oct 2019
 *      Author: Arion
 */




/*
 * CAN frame scheme
 *
 * Idea: Kind of shift-register
 *
 * ID: As per the XBEE defined protocol
 * Byte 0: Hash
 * Byte 1-7: Data
 *
 */
/*
extern CAN_HandleTypeDef hcan1;

CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
uint32_t              TxMailbox;

static void __bus_write64(uint8_t hash, uint8_t* buffer, uint8_t length) {
	static CAN_TxHeaderTypeDef tx_header;
	static uint8_t tx_buffer[8] = {0};
	uint8_t* tx_buffer_pointer = tx_buffer;

	tx_buffer[0] = hash;

	while(length--) {
		*tx_buffer_pointer++ = *buffer++;
	}

	while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)) {} // wait for CAN to be ready

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, TxData, &TxMailbox) == HAL_OK) {
    	can_addMsg((CAN_msg) {data, data_id, timestamp, TxHeader.StdId});
    } else { // something bad happen
    	// not sure what to do
    }
}

static uint32_t __bus_read(void) {
    uint32_t fill_level = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);

    if (fill_level > 0) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

        can_current_msg.data = 0;
        can_current_msg.data += (uint32_t) RxData[0] << 24;
        can_current_msg.data += (uint32_t) RxData[1] << 16;
        can_current_msg.data += (uint32_t) RxData[2] << 8;
		can_current_msg.data += (uint32_t) RxData[3] << 0;

        can_current_msg.id = RxData[4];

        can_current_msg.timestamp = 0;
        can_current_msg.timestamp += (uint32_t) RxData[5] << 16;
        can_current_msg.timestamp += (uint32_t) RxData[6] << 8;
		can_current_msg.timestamp += (uint32_t) RxData[7] << 0;

        can_current_msg.id_CAN = RxHeader.StdId;
    }
    return fill_level;
}*/
