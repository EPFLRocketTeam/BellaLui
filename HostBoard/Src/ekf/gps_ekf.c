/* gps_ekf: TinyEKF test case using You Chong's GPS example:
 * 
 *   http://www.mathworks.com/matlabcentral/fileexchange/31487-extended-kalman-filter-ekf--for-gps
 * 
 * Reads file gps.csv of satellite data and writes file ekf.csv of mean-subtracted estimated positions.
 *
 *
 * References:
 *
 * 1. R G Brown, P Y C Hwang, "Introduction to random signals and applied 
 * Kalman filtering : with MATLAB exercises and solutions",1996
 *
 * 2. Pratap Misra, Per Enge, "Global Positioning System Signals, 
 * Measurements, and Performance(Second Edition)",2006
 * 
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <strings.h>
#include <math.h>

#include "ekf/tinyekf_config.h"
#include "ekf/tiny_ekf.h"
#include "cmsis_os.h"
#include "CAN_communication.h"
#include "Misc/datastructs.h"


#define EKF_PERIOD_MS (100) // [ms] step period

#define earth_radius (6378e3)

double rad2deg(double deg) {
	return deg*3.14/180;
}

float IMUb[6];
float zdata[4];

volatile bool IMU_avail = false;
volatile bool GPS_avail = false;

bool GPS_init = false;
float lat_init = 0;
float lon_init = 0;

bool kalman_handleGPSData(GPS_data gps) {
	/*
	if (!GPS_init) {
		lat_init = gps.lat;
		lon_init = gps.lon;
		GPS_init = true;
	}

	zdata[0] = 0 * rad2deg(gps.lat-lat_init) * earth_radius; // x gps
	zdata[1] = 0 * rad2deg(gps.lon-lon_init) * earth_radius / cos(rad2deg(lat_init)); // y gps
	zdata[2] = 0 * ((float) gps.altitude)/100; // z gps, from cm to m
	zdata[3] = zdata[2]; // z baro...
	GPS_avail = true;
	*/
	return true;
}

bool kalman_handleIMUData(IMU_data imu) {
	IMUb[0] = imu.acceleration.x * 9.81;
	IMUb[1] = imu.acceleration.y * 9.81;
	IMUb[2] = imu.acceleration.z * 9.81;
	IMUb[3] = 0*imu.eulerAngles.x; // actually rotation speed not eulerAngles
	IMUb[4] = 0*imu.eulerAngles.y;
	IMUb[5] = 0*imu.eulerAngles.z;
	IMU_avail = 1;
	return true;
}

bool kalman_handleBaroData(BARO_data data) {
	zdata[3] = data.altitude - data.base_altitude;
	return true;
}


static void init(ekf_t * ekf) {
	// Set Q, see [1]
	/*const float Sf    = 36;
	 const float Sg    = 0.01;
	 const float sigma = 5;         // state transition variance
	 const float Qb[4] = {Sf*T+Sg*T*T*T/3, Sg*T*T/2, Sg*T*T/2, Sg*T};
	 const float Qxyz[4] = {sigma*sigma*T*T*T/3, sigma*sigma*T*T/2, sigma*sigma*T*T/2, sigma*sigma*T};*/

	float Qtmp[81] = { 1.3085e-08, 9.2466e-25, 6.6028e-26, 9.8168e-07,
			6.9357e-23, 4.9523e-24, 0, 0, 0, -6.4641e-26, 1.3083e-08,
			-3.9941e-25, -4.8495e-24, 9.8132e-07, -2.9957e-23, 0, 0, 0,
			7.9888e-25, -5.9826e-25, 1.3081e-08, 5.9934e-23, -4.4875e-23,
			9.8116e-07, 0, 0, 0, .8168e-07, 6.937e-23, 4.9536e-24, 9.8168e-05,
			6.9363e-21, 4.9529e-22, 0, 0, 0, -4.8486e-24, 9.8132e-07,
			-2.996e-23, -4.849e-22, 9.8132e-05, -2.9959e-21, 0, 0, 0,
			5.9918e-23, -4.4872e-23, 9.8116e-07, 5.9926e-21, -4.4874e-21,
			9.8116e-05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0002, 1.4035e-20,
			8.6736e-21, 0, 0, 0, 0, 0, 0, 2.4938e-21, 0.0002, 0, 0, 0, 0, 0, 0,
			0, 9.5501e-21, -1.2369e-20, 0.0002 };
	int i, j;
	for (i = 0; i < 9; i++) {
		ekf->x[i] = 0;
		for (j = 0; j < 9; j++)
			ekf->Q[i][j] = Qtmp[i * 9 + j];
	}

	// initial covariances of state noise, measurement noise
	float P0[9] = { 2, 2, 2, 1, 1, 1, 0.1, 0.1, 0.1 };
	float R0[4] = { 20, 20, 10, 10 }; //accuracy of the GPS and baro

	for (i = 0; i < 9; ++i)
		ekf->P[i][i] = P0[i];

	for (i = 0; i < 4; ++i)
		ekf->R[i][i] = R0[i];

}

void TK_kalman() {
	// Do generic EKF initialization
	ekf_t ekf;
	ekf_init(&ekf, Nsta, Mobs);

	// Do local initialization
	init(&ekf);


	double IMUmd[6];
	float IMUm[6];
	float F11[9][9];
	float dt = ((float) EKF_PERIOD_MS)/1e3; // fix at 10 Hz
	double sp, sr, sy, cp, cr, cy;

	uint32_t start_time=0, now=0, t_wait=0;
	uint32_t iter = 0;
	uint8_t rocket_state = can_getState();
	enum Kalman_state kalman_state = KALMAN_INIT;


	int j, k;
	for (j = 0; j < 9; j++)
		for (k = 0; k < 9; k++)
			F11[j][k] = 0;
	for (j = 0; j < 6; j++) {
		IMUb[j] = 0;
	}

	start_time = HAL_GetTick();


	while (1) {
		rocket_state = can_getState();

		if (IMU_avail == 1) {
			IMU_avail = 0;
			kalman_state = KALMAN_OK;
			//getting the data of the captor in the mapping frame IMUb to IMUm
			sr = sin(ekf.x[6]); //roll
			sp = sin(ekf.x[7]); //pitch
			sy = sin(ekf.x[8]); //yaw
			cr = cos(ekf.x[6]); //roll
			cp = cos(ekf.x[7]); //pitch
			cy = cos(ekf.x[8]); //yaw

			IMUmd[0] = cp * cy * IMUb[0] + (sr * sp * cy + cr * sy) * IMUb[1]
					+ (-cr * sp * cy + sr * sy) * IMUb[2];
			IMUmd[1] = -cp * sy * IMUb[0] + (-sr * sp * sy + cr * cy) * IMUb[1]
					+ (cr * sp * sy + sr * cy) * IMUb[2];
			IMUmd[2] = sp * IMUb[0] - sr * cp * IMUb[1] + cr * cp * IMUb[2];

			IMUmd[3] = cp * cy * IMUb[3] + (sr * sp * cy + cr * sy) * IMUb[4]
					+ (-cr * sp * cy + sr * sy) * IMUb[5];
			IMUmd[4] = -cp * sy * IMUb[3] + (-sr * sp * sy + cr * cy) * IMUb[4]
					+ (cr * sp * sy + sr * cy) * IMUb[5];
			IMUmd[5] = sp * IMUb[3] - sr * cp * IMUb[4] + cr * cp * IMUb[5];

			IMUm[0] = (float) IMUmd[0];
			IMUm[1] = (float) IMUmd[1];
			IMUm[2] = (float) IMUmd[2] - 9.81;
			IMUm[3] = (float) IMUmd[3];
			IMUm[4] = (float) IMUmd[4];
			IMUm[5] = (float) IMUmd[5];

			//fill fx
			ekf.fx[6] = ekf.x[6] + IMUm[3] * dt;
			ekf.fx[7] = ekf.x[7] + IMUm[4] * dt;
			ekf.fx[8] = ekf.x[8] + IMUm[5] * dt;
			ekf.fx[3] = ekf.x[3] + IMUm[0] * dt;
			ekf.fx[4] = ekf.x[4] + IMUm[1] * dt;
			ekf.fx[5] = ekf.x[5] + IMUm[2] * dt;
			ekf.fx[0] = ekf.x[0] + ekf.x[3] * dt;
			ekf.fx[1] = ekf.x[1] + ekf.x[4] * dt;
			ekf.fx[2] = ekf.x[2] + ekf.x[5] * dt;

			//fill F
			F11[0][3] = 1 * dt;
			F11[1][4] = 1 * dt;
			F11[2][5] = 1 * dt;
			F11[3][7] = -IMUm[2] * dt;
			F11[3][8] = IMUm[1] * dt;
			F11[4][6] = IMUm[2] * dt;
			F11[4][8] = -IMUm[0] * dt;
			F11[5][6] = -IMUm[1] * dt;
			F11[5][7] = IMUm[0] * dt;

			mat_exp(F11, ekf.F, 9); //2nd order taylor, exact since F11^3 = 0

			//fill hx
			ekf.hx[0] = ekf.fx[0];
			ekf.hx[1] = ekf.fx[1];
			ekf.hx[2] = ekf.fx[2];
			ekf.hx[3] = ekf.fx[2];

			if (GPS_init && GPS_avail && (iter%1) == 0) { // go for the kalman
				GPS_avail = false;
				//fill H
				ekf.H[0][0] = 1;
				ekf.H[1][1] = 1;
				ekf.H[2][2] = 1;
				ekf.H[3][2] = 1;
				ekf.x[6] = 0; // force orientation pointing up
				ekf.x[7] = 0;
				ekf.x[8] = 0;
				can_setFrame((int32_t) (1000 * zdata[0]), 51, HAL_GetTick());
				can_setFrame((int32_t) (1000 * zdata[1]), 52, HAL_GetTick());
			} else { //simple INS
				//fill H
				ekf.H[0][0] = 0;
				ekf.H[1][1] = 0;
				ekf.H[2][2] = 0;
				ekf.H[3][2] = 1; // includes baro
			}

			ekf_step(&ekf, zdata);
			iter++;


			//send estimate to the CAN
			//can_setFrame((int32_t) (1000 * ekf.x[0]), DATA_ID_KALMAN_X, HAL_GetTick());
			//can_setFrame((int32_t) (1000 * ekf.x[1]), DATA_ID_KALMAN_Y, HAL_GetTick());
			can_setFrame((int32_t) (1000 * ekf.x[2]), DATA_ID_KALMAN_Z, HAL_GetTick());
			//can_setFrame((int32_t) (1000 * ekf.x[3]), DATA_ID_KALMAN_VX, HAL_GetTick());
			//can_setFrame((int32_t) (1000 * ekf.x[4]), DATA_ID_KALMAN_VY, HAL_GetTick());
			can_setFrame((int32_t) (1000 * ekf.x[5]), DATA_ID_KALMAN_VZ, HAL_GetTick());
			//can_setFrame((int32_t) (180 / 3.14 * ekf.x[6]), DATA_ID_KALMAN_ROLL, HAL_GetTick());
			//can_setFrame((int32_t) (180 / 3.14 * ekf.x[7]), DATA_ID_KALMAN_PITCH, HAL_GetTick());
			//can_setFrame((int32_t) (180 / 3.14 * ekf.x[8]), DATA_ID_KALMAN_YAW, HAL_GetTick());
			ekf.x[6] = 0; // force orientation pointing up
			ekf.x[7] = 0;
			ekf.x[8] = 0;
		} else {
			// no IMU data available :sadface:
			kalman_state = KALMAN_NO_IMU;
		}

		// ensure periodicity
		now = HAL_GetTick();
		t_wait = EKF_PERIOD_MS - (now-start_time);
		if (t_wait>0) {
			if (kalman_state != KALMAN_NO_IMU) kalman_state = KALMAN_OK;
			osDelay(t_wait);
			start_time += EKF_PERIOD_MS;
		} else {
			kalman_state = KALMAN_OVERRUN;
			start_time = now; // skipped a tick
		}
		can_setFrame(kalman_state, DATA_ID_KALMAN_STATE, HAL_GetTick());
	}
}
