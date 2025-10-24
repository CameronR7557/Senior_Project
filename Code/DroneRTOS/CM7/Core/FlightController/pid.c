/*
 * pid.c
 *
 *  Created on: Nov 16, 2023
 *      Author: robin
 */

#include "pid.h"
void PID_Init(PIDController* pid, float p_gain, float i_gain, float d_gain)
{
	if(pid != NULL)
	{
		pid->PGain = p_gain;
		pid->IGain = i_gain;
		pid->DGain = d_gain;
	}
}


float PID_Update(PIDController* pid, float measurement, float setpoint, float sampleTime)
{
	float out = 0.0f;
	float prev_error = pid->error;
	float IUpperLimit = 0.0;
	float ILowerLimit = 0.0;

	pid->error = setpoint - measurement;
	pid->PTerm = pid->PGain * pid->error;
	pid->ITerm = ((pid->IGain)*sampleTime*0.5f) * (pid->error + prev_error) + pid->ITerm;
	//For derivative-on-measurement, a - needs to be multiplied into measurement-prev_measurement to make it the same as error - prev error
	//pid->DTerm = (2.0f*(pid->DGain)/(2.0f*(pid->tau) + sampleTime)) * -(measurement - pid->prev_measurement) + ((2.0f*(pid->tau) - sampleTime)/(2.0f*(pid->tau) + sampleTime)) * pid->DTerm;
	pid->DTerm = (2.0f * pid->DGain * (pid->prev_measurement - measurement)
			   + (2.0f * pid->tau - sampleTime) * pid->DTerm) / (2.0f * pid->tau + sampleTime);
	//pid->DTerm = (2*(pid->DGain)/(2*(pid->tau) + sampleTime)) * (pid->error - prev_error) + ((2*(pid->tau) - sampleTime)/(2*(pid->tau) + sampleTime)) * pid->DTerm;//Derivative on error - not preferable
	pid->prev_measurement = measurement;
	//I Clamping
	if(pid->upperLimit > pid->PTerm)
		IUpperLimit = pid->upperLimit - pid->PTerm;
	else
		IUpperLimit = 0.0f;

	if(pid->lowerLimit < pid->PTerm)
		ILowerLimit = pid->lowerLimit - pid->PTerm;
	else
		ILowerLimit = 0.0f;

	if(pid->ITerm > IUpperLimit)
		pid->ITerm = IUpperLimit;
	else if(pid->ITerm < ILowerLimit)
		pid->ITerm = ILowerLimit;

	//Calculate Output
	out = pid->PTerm + pid->ITerm + pid->DTerm;

	//Output Clamping
	if(out > pid->upperLimit)
		out = pid->upperLimit;
	else if(out < pid->lowerLimit)
		out = pid->lowerLimit;

	return out;
}

