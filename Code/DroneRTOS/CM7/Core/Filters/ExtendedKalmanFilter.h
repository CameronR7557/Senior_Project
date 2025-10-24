/*
 * ExtendedKalmanFilter.h
 *	github.com/pms67/HadesFCS/blob/master/Firmware/F4Quad/Core/Src/KalmanRollPitch.
 */

#ifndef FILTERS_EXTENDEDKALMANFILTER_H_
#define FILTERS_EXTENDEDKALMANFILTER_H_
#include "../FlightController/flightcontroller.h"
#include "stdint.h"
#include "math.h"

#define EKF_P_INIT 0.1f//Modified this, was originally 0.1. Supposedly, this is how confident you are in initial estimation
#define EKF_N_GYR 0.00000191056f	//Q init -- May need to tune these. Could help to look in datasheet
#define EKF_N_ACC 0.000067666f		//R init

typedef struct {
	float P[4];
	float Q[2];
	float R[3];
	float roll;
	float pitch;
}KalmanFilterRollPitch;

void EKF_Init(KalmanFilterRollPitch* filter, float Pinit, float* Q, float* R);
void EKF_Predict(KalmanFilterRollPitch* filter, struct bmi08x_sensor_data_f* gyro, float delta_t);
void EKF_Update(KalmanFilterRollPitch* filter, struct bmi08x_sensor_data_f* acc, float Va);
float VelocityEstimate(float* vel, struct bmi08x_sensor_data_f* acc, struct bmi08x_sensor_data_f* gyro, float deltaT, float roll, float pitch);

#endif /* FILTERS_EXTENDEDKALMANFILTER_H_ */
