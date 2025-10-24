/*
 * ExtendedKalmanFilter.c
 *  github.com/pms67/HadesFCS/blob/master/Firmware/F4Quad/Core/Src/KalmanRollPitch.
 */
#include "ExtendedKalmanFilter.h"

void EKF_Init(KalmanFilterRollPitch* filter, float Pinit, float* Q, float* R)
{
	//Init estimates are 0
	filter->roll = 0.0f;
	filter->pitch = 0.0f;

	//Fill in diagonals of P
	filter->P[0] = Pinit;
	filter->P[1] = 0.0f;
	filter->P[2] = 0.0f;
	filter->P[3] = Pinit;

	//Initialize diagonal matrices (Set to arrays b/c assuming only diagonals have values)
	filter->Q[0] = Q[0];
	filter->Q[1] = Q[1];
	filter->R[0] = R[0];
	filter->R[1] = R[1];
	filter->R[2] = R[2];

}
void EKF_Predict(KalmanFilterRollPitch* filter, struct bmi08x_sensor_data_f* gyro, float delta_t)
{
	//Store gyro readings and convert to radians/s
	float p = gyro->x * DEG_TO_RAD;
	float q = -gyro->y * DEG_TO_RAD;
	float r = -gyro->z * DEG_TO_RAD;

	float sin_roll = sinf(filter->roll);
	float cos_roll = cosf(filter->roll);
	float tan_pitch = tanf(filter->pitch);

	//Update estimates with Euler Integration (prev + dt*(state trans func))
	filter->roll = filter->roll + delta_t*(p + tan_pitch*(q*sin_roll + r*cos_roll));
	filter->pitch = filter->pitch + delta_t*(q*cos_roll - r*sin_roll);

	//Update trig calculations
	float sin_pitch = sinf(filter->pitch);
	float cos_pitch = cosf(filter->pitch);
	sin_roll = sinf(filter->roll);
	cos_roll = cosf(filter->roll);
	tan_pitch = sin_pitch/cos_pitch;
	float sec2_pitch = tan_pitch * tan_pitch + 1.0f;//tan^2 + 1 = sec^2

	//Update jacobian of state trans function f(x,u)
	float A[4] = {tan_pitch*(q*cos_roll - sin_roll*r), sec2_pitch*(q*sin_roll + r*cos_roll), (-q*sin_roll - r*cos_roll), 0.0f};

	/* Update covariance matrix P+ = P- + T * (A*P- + P-*A' + Q) */
	float Ptmp[4] = { delta_t*(filter->Q[0]      + 2.0f*A[0]*filter->P[0] + A[1]*filter->P[1] + A[1]*filter->P[2]), delta_t*(A[0]*filter->P[1] + A[2]*filter->P[0] + A[1]*filter->P[3] + A[3]*filter->P[1]),
					  delta_t*(A[0]*filter->P[2] + A[2]*filter->P[0]   + A[1]*filter->P[3] + A[3]*filter->P[2]),    delta_t*(filter->Q[1]      + A[2]*filter->P[1] + A[2]*filter->P[2] + 2.0f*A[3]*filter->P[3]) };

	filter->P[0] = filter->P[0] + Ptmp[0];
	filter->P[1] = filter->P[1] + Ptmp[1];
	filter->P[2] = filter->P[2] + Ptmp[2];
	filter->P[3] = filter->P[3] + Ptmp[3];

}
void EKF_Update(KalmanFilterRollPitch* filter, struct bmi08x_sensor_data_f* acc, float Va)
{
	float ax = acc->x;
	float ay = -acc->y;
	float az = -acc->z;

	/* Compute common trig terms */
	float sin_roll = sin(filter->roll);
	float cos_roll = cos(filter->roll);
	float sin_pitch = sin(filter->pitch);
	float cos_pitch = cos(filter->pitch);

	/* Output funcos_pitchion h(x,u) */
	float h[3] = {GRAVITY * sin_pitch, -GRAVITY * cos_pitch * sin_roll, -GRAVITY * cos_pitch * cos_roll};//Check this ---------------------

	float C[6] = {0.0f, GRAVITY * cos_pitch, -GRAVITY * cos_roll * cos_pitch, GRAVITY * sin_roll * sin_pitch, GRAVITY * sin_roll * cos_pitch, GRAVITY * cos_roll * sin_pitch};


	/* Kalman gain K = P * C' / (C * P * C' + R) */
	float G[9] = { filter->P[3]*C[1]*C[1] + filter->R[0],        C[1]*C[2]*filter->P[2] + C[1]*C[3]*filter->P[3],                                                   C[1]*C[4]*filter->P[2] + C[1]*C[5]*filter->P[3],
				   C[1]*(C[2]*filter->P[1] + C[3]*filter->P[3]), filter->R[1] + C[2]*(C[2]*filter->P[0] + C[3]*filter->P[2]) + C[3]*(C[2]*filter->P[1] + C[3]*filter->P[3]), C[4]*(C[2]*filter->P[0] + C[3]*filter->P[2]) + C[5]*(C[2]*filter->P[1] + C[3]*filter->P[3]),
	               C[1]*(C[4]*filter->P[1] + C[5]*filter->P[3]), C[2]*(C[4]*filter->P[0] + C[5]*filter->P[2]) + C[3]*(C[4]*filter->P[1] + C[5]*filter->P[3]),             filter->R[2] + C[4]*(C[4]*filter->P[0] + C[5]*filter->P[2]) + C[5]*(C[4]*filter->P[1] + C[5]*filter->P[3]) };

	float Gdet = (G[0]*G[4]*G[8] - G[0]*G[5]*G[7] - G[1]*G[3]*G[8] + G[1]*G[5]*G[6] + G[2]*G[3]*G[7] - G[2]*G[4]*G[6]);

	/* Ensure matrix is non-singular */
	if (Gdet < -0.000001f || Gdet > 0.000001f) {
		float Gdetinv = 1.0f / Gdet;

		float Ginv[9] = { Gdetinv * (G[4]*G[8] - G[5]*G[7]), -Gdetinv * (G[1]*G[8] - G[2]*G[7]),  Gdetinv * (G[1]*G[5] - G[2]*G[4]),
						 -Gdetinv * (G[3]*G[8] - G[5]*G[6]),  Gdetinv * (G[0]*G[8] - G[2]*G[6]), -Gdetinv * (G[0]*G[5] - G[2]*G[3]),
						  Gdetinv * (G[3]*G[7] - G[4]*G[6]), -Gdetinv * (G[0]*G[7] - G[1]*G[6]),  Gdetinv * (G[0]*G[4] - G[1]*G[3]) };

		float K[6] = { Ginv[3]*(C[2]*filter->P[0] + C[3]*filter->P[1]) + Ginv[6]*(C[4]*filter->P[0] + C[5]*filter->P[1]) + C[1]*Ginv[0]*filter->P[1], Ginv[4]*(C[2]*filter->P[0] + C[3]*filter->P[1]) + Ginv[7]*(C[4]*filter->P[0] + C[5]*filter->P[1]) + C[1]*Ginv[1]*filter->P[1], Ginv[5]*(C[2]*filter->P[0] + C[3]*filter->P[1]) + Ginv[8]*(C[4]*filter->P[0] + C[5]*filter->P[1]) + C[1]*Ginv[2]*filter->P[1],
					   Ginv[3]*(C[2]*filter->P[2] + C[3]*filter->P[3]) + Ginv[6]*(C[4]*filter->P[2] + C[5]*filter->P[3]) + C[1]*Ginv[0]*filter->P[3], Ginv[4]*(C[2]*filter->P[2] + C[3]*filter->P[3]) + Ginv[7]*(C[4]*filter->P[2] + C[5]*filter->P[3]) + C[1]*Ginv[1]*filter->P[3], Ginv[5]*(C[2]*filter->P[2] + C[3]*filter->P[3]) + Ginv[8]*(C[4]*filter->P[2] + C[5]*filter->P[3]) + C[1]*Ginv[2]*filter->P[3] };

		/* Update covariance matrix P++ = (I - K * C) * P+ */
		float Ptmp[4];
		Ptmp[0] = -filter->P[2]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - filter->P[0]*(C[2]*K[1] + C[4]*K[2] - 1.0f);
		Ptmp[1] = -filter->P[3]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - filter->P[1]*(C[2]*K[1] + C[4]*K[2] - 1.0f);
		Ptmp[2] = -filter->P[2]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f) - filter->P[0]*(C[2]*K[4] + C[4]*K[5]);
		Ptmp[3] = -filter->P[3]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0f) - filter->P[1]*(C[2]*K[4] + C[4]*K[5]);

		filter->P[0] = filter->P[0] + Ptmp[0]; filter->P[1] = filter->P[1] + Ptmp[1];
		filter->P[2] = filter->P[2] + Ptmp[2]; filter->P[3] = filter->P[3] + Ptmp[3];

		/* Update sin_pitchate esin_pitchimate x++ = x+ + K * (y - h) */
		filter->roll   = filter->roll   + K[0] * (ax - h[0]) + K[1] * (ay - h[1]) + K[2] * (az - h[2]);
		filter->pitch = filter->pitch + K[3] * (ax - h[0]) + K[4] * (ay - h[1]) + K[5] * (az - h[2]);

	}
}

float VelocityEstimate(float* vel, struct bmi08x_sensor_data_f* acc, struct bmi08x_sensor_data_f* gyro, float deltaT, float roll, float pitch)
{
	//gyro p q r -> x y z
	//Need to subtract gravity from acc
	vel[0] = vel[0] + deltaT*(gyro->z*vel[1] - gyro->y*vel[2] + acc->x);
	vel[1] = vel[1] + deltaT*(-gyro->z*vel[0] + gyro->x*vel[2] + acc->y);
	vel[2] = vel[2] + deltaT*(-gyro->x*vel[0] + gyro->y*vel[1] + acc->z);
	return sqrtf(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
}


