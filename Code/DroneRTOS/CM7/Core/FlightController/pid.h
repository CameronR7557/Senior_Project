/*
 * pid.h
 *
 *  Created on: Nov 16, 2023
 *      Author: robin
 */

#ifndef FLIGHTCONTROLLER_PID_H_
#define FLIGHTCONTROLLER_PID_H_
#include "stdint.h"
#include "stddef.h"


typedef struct {
	float PGain;
	float IGain;
	float DGain;

	float upperLimit;
	float lowerLimit;

	float tau; //Filter time constant. Lower tau -> less derivative filtering
	float error;
	float PTerm;
	float ITerm;
	float DTerm;
	float prev_measurement;
}PIDController;


void PID_Init(PIDController* pid, float p_gain, float i_gain, float d_gain);
float PID_Update(PIDController* pid, float measurement, float setpoint, float sampleTime);

#endif /* FLIGHTCONTROLLER_PID_H_ */
