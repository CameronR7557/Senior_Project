/*
 * flightcontroller.c
 *
 *  Created on: Nov 2, 2023
 *      Author: robin
 */

#include "flightcontroller.h"



//PID Controller:
/*
 * Things to consider:
 * Control roll and pitch to angles based on JS input.
 * May not need to control Yaw angle at all, maybe try controlling to a rotation speed (but consider: how does it yaw if roll and pitch are not 0? May need to know the yaw angle)
 * Be aware that gimbal lock occurs at 90-deg, so need to set hard limits before that
 * Need to maintain total thrust by distributing sums and differences of throttle changes to all four motors
 * */

