/*
 * flightcontroller.h
 *
 *  Created on: Nov 2, 2023
 *      Author: robin
 */

#ifndef FLIGHTCONTROLLER_FLIGHTCONTROLLER_H_
#define FLIGHTCONTROLLER_FLIGHTCONTROLLER_H_

#include "bmi08x_defs.h"
#include "math.h"
#include "../Filters/FIRFilter.h"
#include "../Filters/ExtendedKalmanFilter.h"
#include "pid.h"

#define PID_MULT 				4.0f
//For One-layer controllers:
//#define MAX_ROLL_ANGLE 		    25.0f * PID_MULT//Roll and Pitch are multiplied by 10, should control to +/- 35 deg
//#define MAX_PITCH_ANGLE 		25.0f * PID_MULT

//For two-layer controllers:
#define MAX_ROLL_ANGLE 		    25.0f
#define MAX_PITCH_ANGLE 		25.0f


#define MAX_ROLL_RATE		    120.0f
#define MAX_PITCH_RATE  		120.0f
#define MAX_YAW_RATE 		    120.0f
#define ANGLE_TO_RATE_CONV		(MAX_ROLL_RATE / MAX_ROLL_ANGLE)
#define MAX_THROTTLE	        1647 //(.8 * 2000) + 48 (max throttle = 2047, min = 48).
							 	 	 //Gives 20% throttle to work with for PID control
#define MAX_JOYSTICK_VAL 		1024.0f

#define ALPHA 			0.05F
#define GRAVITY 		9.8067F
#define PI				3.141592654F
#define RAD_TO_DEG		57.29577951F//((float)(180/PI))
#define DEG_TO_RAD		(float)(PI/180.0F) //PI/180

typedef struct
{
	uint16_t throttle;
	int16_t yaw;
	int16_t pitch;
	int16_t roll;

}motor_values;

typedef struct
{
	int16_t motor1;
	int16_t motor2;
	int16_t motor3;
	int16_t motor4;

}DShotVals;

typedef struct
{
	float yaw;
	float pitch;
	float roll;
	float rollRate;
	float pitchRate;
	float yawRate;
}DroneRotations;



#endif /* FLIGHTCONTROLLER_FLIGHTCONTROLLER_H_ */
