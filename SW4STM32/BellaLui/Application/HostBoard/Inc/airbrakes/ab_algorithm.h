/*
 * ab_algorithm.h
 *
 *  Created on: 26 Jun 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_AIRBRAKES_AB_ALGORITHM_H_
#define APPLICATION_HOSTBOARD_INC_AIRBRAKES_AB_ALGORITHM_H_


#include "eiger_algorithm.h"
#include "bellalui_algorithm.h"

#define AB_BELLALUI

#ifdef AB_EIGER
#define angle_tab eiger_angle_tab
#elif defined(AB_BELLALUI)
#define angle_tab bellalui_angle_tab
#endif


#endif /* APPLICATION_HOSTBOARD_INC_AIRBRAKES_AB_ALGORITHM_H_ */
