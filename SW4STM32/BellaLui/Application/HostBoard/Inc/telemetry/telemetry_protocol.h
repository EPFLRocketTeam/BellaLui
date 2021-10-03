/* telemetry_protocol.h
 *
 *  Created on: 19 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef TELEMETRY_TELEMETRY_PROTOCOL_H_
#define TELEMETRY_TELEMETRY_PROTOCOL_H_

enum DatagramPayloadType {
	GPS_PACKET = 0x01,
	STATUS_PACKET = 0x02,
	TELEMETRY_PACKET = 0x03,
	DEBUG_PACKET = 0x04,
	MYSTERIOUS_PACKET = 0x05,
	AIRBRAKES_PACKET = 0x06,
	IGNITION_PACKET = 0x07,
	ORDER_PACKET = 0x09,
	GSE_STATE_PACKET = 0x0A,
	ECHO_PACKET = 0x0B,
	PROPULSION_COMMAND_PACKET = 0x0C,
	PROPULSION_DATA_PACKET = 0x0D,
	TVC_COMMAND_PACKET = 0x0E,
	TVC_STATUS_PACKET = 0x0F
};

#define HEADER_PREAMBLE_FLAG 0x55
#define CONTROL_FLAG 0xFF
#define START_DELIMITER_SIZE 1
#define MSB_SIZE 1
#define LSB_SIZE 1
#define CHECKSUM_SIZE 2 // TODO: 1 or 2 ?????
#define ACTIVATE_DATAGRAM_CHECKSUM 1

// Packet Header size in bytes
#define XBEE_OPTIONS_SIZE 14
#define XBEE_RECEIVED_OPTIONS_SIZE 13
#define DATAGRAM_ID_SIZE 1
#define PREFIX_EPFL_SIZE 4
#define PACKET_NUMBER_SIZE 1
#define TIMESTAMP_SIZE 4
#define SEQ_NUMBER_SIZE 4
#define HEADER_SIZE (XBEE_OPTIONS_SIZE + DATAGRAM_ID_SIZE + PREFIX_EPFL_SIZE + TIMESTAMP_SIZE + SEQ_NUMBER_SIZE)
#define RECEIVED_HEADER_SIZE (XBEE_RECEIVED_OPTIONS_SIZE + DATAGRAM_ID_SIZE + PREFIXE_EPFL_SIZE + TIMESTAMP_SIZE + SEQ_NUMBER_SIZE)

// Sections sizes in bytes
/*
#define PREAMBLE_SIZE 4
#define CONTROL_FLAG_SIZE 1
#define CHECKSUM_SIZE 1
*/
#define TOTAL_DATAGRAM_HEADER (DATAGRAM_ID_SIZE + PREFIX_EPFL_SIZE + TIMESTAMP_SIZE + SEQ_NUMBER_SIZE)
#define TOTAL_DATAGRAM_OVERHEAD (TOTAL_DATAGRAM_HEADER + ACTIVATE_DATAGRAM_CHECKSUM * CHECKSUM_SIZE)

#define SENSOR_DATAGRAM_PAYLOAD_SIZE 40
#define GPS_DATAGRAM_PAYLOAD_SIZE 17 // TODO: 13 or 17 ??
#define TELEMETRY_RAW_DATAGRAM_PAYLOAD_SIZE 24
#define TELEMETRY_FILTERED_DATAGRAM_PAYLOAD_SIZE 24
#define PROPULSION_DATAGRAM_PAYLOAD_SIZE 22
#define TVC_STATUS_PAYLOAD_SIZE 8
#define STATUS_DATAGRAM_PAYLOAD_SIZE 6
#define AB_DATAGRAM_PAYLOAD_SIZE 4

#endif /* TELEMETRY_TELEMETRY_PROTOCOL_H_ */
