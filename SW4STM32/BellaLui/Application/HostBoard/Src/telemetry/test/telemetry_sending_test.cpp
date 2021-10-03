/*
 * telemetry_sending_test.cpp
 *
 *  Created on: 16 Mar 2021
 *      Author: Oliver
 */

#include <gtest/gtest.h>
#include "telemetry/telemetry_sending.h"
#include "telemetry/queue/Queue.h"
#include "telemetry/queue/StlMessageQueue.h"
#include "telemetry/test/datagram_check.hpp"
#include "Embedded/can.h"
#include <stdio.h>


TEST(TelemetryTest, TelemetrySending_NoQueue) {
	uint32_t ts = 1234;

	GPS_data gps = {0.9, 42.42, 5.5, 5025, 4};
	EXPECT_FALSE(telemetrySendGPS(ts, gps));

	IMU_data imu = { { 1.1, 2.2, 3.3 }, { 4.4, 5.5, 6.6 } };
	EXPECT_FALSE(telemetrySendIMU(ts, imu));

	BARO_data baro = { 15.5, 1.05, 5024.6, 0, 0 };
	EXPECT_FALSE(telemetrySendBaro(ts, baro));

	float angle = 12.05;
	EXPECT_FALSE(telemetrySendAirbrakesAngle(ts, angle));

	uint8_t id = 1;
	float val = 10.1;
	uint8_t state = 0x08;
	EXPECT_FALSE(telemetrySendState(ts, id, val, state));

	PropulsionData prop = {1001, 1002, 20, -45, 12, 8, -42};
	EXPECT_FALSE(telemetrySendPropulsionData(ts, &prop));
}

TEST(TelemetryTest, TelemetrySending_Gps) {
	StlMessageQueue<Telemetry_Message> queue(64);
	registerSendQueue(&queue);

	// first push is successful
	uint32_t ts1 = 1234;
	GPS_data gps1 = {0.9, 42.42, 5.5, 5025, 4};
	EXPECT_TRUE(telemetrySendGPS(ts1, gps1));
	EXPECT_EQ(queue.count(), 1);

	// second push comes too early -> refused
	uint32_t ts2 = 2345;
	GPS_data gps2 = {0.8, 43.42, 4.4, 2550, 2};
	usleep(GPS_TIMEMIN * 1000 / 2);
	EXPECT_FALSE(telemetrySendGPS(ts2, gps2));
	EXPECT_EQ(queue.count(), 1);

	// third try is successful
	usleep(GPS_TIMEMIN * 1000);
	EXPECT_TRUE(telemetrySendGPS(ts2, gps2));
	EXPECT_EQ(queue.count(), 2);

	// check contents of queue
	Telemetry_Message *msg;
	EXPECT_TRUE(queue.pop(&msg, 0));
	checkGpsDatagram(msg, ts1, 6, gps1);

	EXPECT_TRUE(queue.pop(&msg, 0));
	checkGpsDatagram(msg, ts2, 7, gps2);

	EXPECT_EQ(queue.count(), 0);
}

TEST(TelemetryTest, TelemetrySending_ImuBaro) {
	StlMessageQueue<Telemetry_Message> queue(64);
	registerSendQueue(&queue);

	// first push IMU is successful
	uint32_t ts1 = 1234;
	IMU_data imu = { { 1.1, 2.2, 3.3 }, { 4.4, 5.5, 6.6 } };
	EXPECT_TRUE(telemetrySendIMU(ts1, imu));
	EXPECT_EQ(queue.count(), 1);

	// second push Baro comes too early -> refused
	uint32_t ts2 = 2345;
	BARO_data baro = { 15.5, 1.05, 5024.6, 0, 0 };
	usleep(TELE_TIMEMIN * 1000 / 3);
	EXPECT_FALSE(telemetrySendBaro(ts2, baro));
	EXPECT_EQ(queue.count(), 1);

	// third push IMU refused
	usleep(TELE_TIMEMIN * 1000 / 3);
	EXPECT_FALSE(telemetrySendIMU(ts1, imu));
	EXPECT_EQ(queue.count(), 1);

	// fourth try Baro successful
	usleep(TELE_TIMEMIN * 1000);
	EXPECT_TRUE(telemetrySendBaro(ts2, baro));
	EXPECT_EQ(queue.count(), 2);

	// check contents of queue
	Telemetry_Message *msg;
	EXPECT_TRUE(queue.pop(&msg, 0));
	checkTelemetryDatagram(msg, ts1, 8, imu, baro, can_getSpeed(), can_getAltitude());

	EXPECT_TRUE(queue.pop(&msg, 0));
	checkTelemetryDatagram(msg, ts2, 9, imu, baro, can_getSpeed(), can_getAltitude());

	EXPECT_EQ(queue.count(), 0);
}

TEST(TelemetryTest, TelemetrySending_Airbrakes) {
	StlMessageQueue<Telemetry_Message> queue(64);
	registerSendQueue(&queue);

	// first push is successful
	uint32_t ts1 = 3456;
	float angle1 = 12.05;
	EXPECT_TRUE(telemetrySendAirbrakesAngle(ts1, angle1));
	EXPECT_EQ(queue.count(), 1);

	// second push comes too early -> refused
	uint32_t ts2 = 2345;
	float angle2 = 13.05;
	usleep(AB_TIMEMIN * 1000 / 2);
	EXPECT_FALSE(telemetrySendAirbrakesAngle(ts2, angle2));
	EXPECT_EQ(queue.count(), 1);

	// third try is successful
	usleep(AB_TIMEMIN * 1000);
	EXPECT_TRUE(telemetrySendAirbrakesAngle(ts2, angle2));
	EXPECT_EQ(queue.count(), 2);

	// check contents of queue
	Telemetry_Message *msg;
	EXPECT_TRUE(queue.pop(&msg, 0));
	checkAirbrakesDatagram(msg, ts1, 10, angle1);

	EXPECT_TRUE(queue.pop(&msg, 0));
	checkAirbrakesDatagram(msg, ts2, 11, angle2);

	EXPECT_EQ(queue.count(), 0);
}

TEST(TelemetryTest, TelemetrySending_State) {
	StlMessageQueue<Telemetry_Message> queue(64);
	registerSendQueue(&queue);

	// first push is successful
	uint32_t ts1 = 4567;
	uint8_t id1 = 1;
	float val1 = 10.1;
	uint8_t state1 = 0x08;
	EXPECT_TRUE(telemetrySendState(ts1, id1, val1, state1));
	EXPECT_EQ(queue.count(), 1);

	// second push comes too early -> refused
	uint32_t ts2 = 5678;
	uint8_t id2 = 2;
	float val2 = 20.2;
	uint8_t state2 = 0x09;
	usleep(STATE_TIMEMIN * 1000 / 2);
	EXPECT_FALSE(telemetrySendState(ts2, id2, val2, state2));
	EXPECT_EQ(queue.count(), 1);

	// third try is successful
	usleep(STATE_TIMEMIN * 1000);
	EXPECT_TRUE(telemetrySendState(ts2, id2, val2, state2));
	EXPECT_EQ(queue.count(), 2);

	// check contents of queue
	Telemetry_Message *msg;
	EXPECT_TRUE(queue.pop(&msg, 0));
	checkStateDatagram(msg, ts1, 12, id1, val1, state1);

	EXPECT_TRUE(queue.pop(&msg, 0));
	checkStateDatagram(msg, ts2, 13, id2, val2, state2);

	EXPECT_EQ(queue.count(), 0);
}

TEST(TelemetryTest, TelemetrySending_Propulsion) {
	StlMessageQueue<Telemetry_Message> queue(64);
	registerSendQueue(&queue);

	// first push is successful
	uint32_t ts1 = 5678;
	PropulsionData prop1 = {1001, 1002, 20, -45, 12, 8, -42};
	EXPECT_TRUE(telemetrySendPropulsionData(ts1, &prop1));
	EXPECT_EQ(queue.count(), 1);

	// second push comes too early -> refused
	uint32_t ts2 = 6789;
	PropulsionData prop2 = {1003, 1004, 22, -42, 21, 9, -35};
	usleep(PROP_DATA_TIMEMIN * 1000 / 2);
	EXPECT_FALSE(telemetrySendPropulsionData(ts2, &prop2));
	EXPECT_EQ(queue.count(), 1);

	// third try is successful
	usleep(PROP_DATA_TIMEMIN * 1000);
	EXPECT_TRUE(telemetrySendPropulsionData(ts2, &prop2));
	EXPECT_EQ(queue.count(), 2);

	// check contents of queue
	Telemetry_Message *msg;
	EXPECT_TRUE(queue.pop(&msg, 0));
	checkPropulsionDatagram(msg, ts1, 14, prop1);

	EXPECT_TRUE(queue.pop(&msg, 0));
	checkPropulsionDatagram(msg, ts2, 15, prop2);

	EXPECT_EQ(queue.count(), 0);
}

TEST(TelemetryTest, TelemetrySending_TVC) {
	StlMessageQueue<Telemetry_Message> queue(64);
	registerSendQueue(&queue);

	// first push is successful
	uint32_t ts1 = 2354;
	TVCStatus tvc1 = {95, 1};
	EXPECT_TRUE(telemetrySendTVCStatus(ts1, &tvc1));
	EXPECT_EQ(queue.count(), 1);

	// second push comes too early -> refused
	uint32_t ts2 = 3465;
	TVCStatus tvc2 = {90, 2};
	usleep(TVC_STATUS_TIMEMIN * 1000 / 2);
	EXPECT_FALSE(telemetrySendTVCStatus(ts2, &tvc2));
	EXPECT_EQ(queue.count(), 1);

	// third try is successful
	usleep(TVC_STATUS_TIMEMIN * 1000);
	EXPECT_TRUE(telemetrySendTVCStatus(ts2, &tvc2));
	EXPECT_EQ(queue.count(), 2);

	// check contents of queue
	Telemetry_Message *msg;
	EXPECT_TRUE(queue.pop(&msg, 0));
	checkTVCDatagram(msg, ts1, 16, tvc1);

	EXPECT_TRUE(queue.pop(&msg, 0));
	checkTVCDatagram(msg, ts2, 17, tvc2);

	EXPECT_EQ(queue.count(), 0);
}

