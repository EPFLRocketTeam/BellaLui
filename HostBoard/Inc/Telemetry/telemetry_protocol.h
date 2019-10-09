/*
 * telemetry_protocol.h
 *
 *  Created on: 19 Apr 2018
 *      Author: Clément Nussbaumer
 */

#ifndef TELEMETRY_TELEMETRY_PROTOCOL_H_
#define TELEMETRY_TELEMETRY_PROTOCOL_H_

enum DatagramPayloadType {
    TELEMETRY = 0x00, EVENT = 0x01, CONTROL = 0x02, GPS = 0x03, TELEMETRY_ERT18 = 0X04
};


#define HEADER_PREAMBLE_FLAG 0x55
#define CONTROL_FLAG 0xFF

// Field sizes in bytes
#define SEQUENCE_NUMBER_SIZE 4
#define PAYLOAD_TYPE_SIZE 1

// Sections sizes in bytes
#define PREAMBLE_SIZE 4
#define HEADER_SIZE (SEQUENCE_NUMBER_SIZE + PAYLOAD_TYPE_SIZE)
#define CONTROL_FLAG_SIZE 1
#define CHECKSUM_SIZE 2
#define TOTAL_DATAGRAM_OVERHEAD (PREAMBLE_SIZE + HEADER_SIZE + CONTROL_FLAG_SIZE + CHECKSUM_SIZE)

#define SENSOR_DATAGRAM_PAYLOAD_SIZE 40
#define GPS_DATAGRAM_PAYLOAD_SIZE 21

#endif /* TELEMETRY_TELEMETRY_PROTOCOL_H_ */
