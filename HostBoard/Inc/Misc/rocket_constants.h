/*
 * rocket_constants.h
 *
 *  Created on: 24 Apr 2018
 *      Author: ebrunner
 */

/*
 * Description : This file gathers all rocket specific constants that must be checked each time a
 * a design change occurs such as changing motor, adding mass, launching from a different altitude,...
 * It also includes calibration data such initial altitude.
 *
 * For a mass or motor change, check :
 * 	- ROCKET_CST_LAUNCH_TRIG_ACCEL
 * 	- ROCKET_CST_MIN_TRIG_AGL
 *
 */

#ifndef MISC_ROCKET_CONSTANTS_H_
#define MISC_ROCKET_CONSTANTS_H_

/*
 * ROCKET PARAMETERS
 */

#define ROCKET_CST_LIFTOFF_TRIG_ACCEL 2 // acceleration lift-off detection trigger [g]
#define ROCKET_CST_MIN_TRIG_AGL 300 // min altitude above ground level to allow apogee detection [m]
#define ROCKET_CST_MOTOR_BURNTIME 5600 // motor burn time [ms]
#define ROCKET_CST_REC_SECONDARY_ALT 150 // altitude of secondary recovery event [m]
#define LIFTOFF_DETECTION_DELAY 500 // delay to trigger the liftoff event [ms]

/*
 * STATE MACHINE PARAMETERS
 */

#define APOGEE_BUFFER_SIZE 100 // Number of descending altitude events before the apogee detection is triggered
#define APOGEE_ALT_DIFF 1 // meters below the apogee that allow the state to be triggered
#define APOGEE_MUTE_TIME 5000 // sensor mute time in ms such that the over-pressure of ejection doesn't trigger a state by accident
#define SECONDARY_BUFFER_SIZE 5 // Number of descending altitude events before the secondary recovery altitude detection is triggered
#define TOUCHDOWN_DELAY_TIME 5000 // delay time in ms between two evaluations of the touch-down event
#define TOUCHDOWN_ALT_DIFF 2 // touch-down altitude offset in which the rocket is considered as static
#define TOUCHDOWN_BUFFER_SIZE 5 // Number of static altitude events before the touch-down detection is triggered

/*
 * CALIBRATION DATA
 */
#define CALIB_BARO_BUFFER_SIZE 50 // Number of measurement values taken by the calibration routine to evaluate intial altitude


#define ADJUSTED_SEA_LEVEL_PRESSURE 1018.6
#define AIR_DENSITY 1.204


#endif /* MISC_ROCKET_CONSTANTS_H_ */
