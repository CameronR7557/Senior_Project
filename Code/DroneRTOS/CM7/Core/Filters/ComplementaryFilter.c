
#include "ComplementaryFilter.h"

void ComplementaryFilter(struct bmi08x_sensor_data_f* acc, struct bmi08x_sensor_data_f* gyro, struct eulerAngles* EulerAngles)
{
	float delta_t = 0.005f; //5ms
	//Estimate values for acc and gyro
	static struct eulerAngles gyro_est = {0, 0, 0};
	static struct eulerAngles acc_est = {0, 0, 0};
	static struct eulerAngles gyro_rates = {0, 0, 0};

	//May want to subtract off the gravity (may or may not work)
	//acc values from IMU are in m/s^2. Gyro values are in deg/s

	//Gets estimated roll and pitch angles from acc in rads
	acc_est.roll = atanf((acc->y)/(acc->z));
	acc_est.pitch = asinf((acc->x)/GRAVITY);


	//Convert gyro data to RAD/s to keep units consistent
	gyro->x = (gyro->x) * DEG_TO_RAD;
	gyro->y = -(gyro->y) * DEG_TO_RAD;//Changed to negative z and y to match axis of model (May not be correct, test it)
	gyro->z = -(gyro->z) * DEG_TO_RAD;

	//Get estimated roll rate using previous EulerAngle estimate for roll and pitch and current readings
	gyro_rates.roll = (gyro->x + tanf(gyro_est.pitch) * ((gyro->y)*sinf(gyro_est.roll) + (gyro->z)*cosf(gyro_est.roll)));//Phi-dot
	gyro_rates.pitch = ((gyro->y)*cosf(gyro_est.roll) - (gyro->z)*sinf(gyro_est.roll));//Theta-dot


	//Complementary Filter:
	//Find new gyro roll and pitch angle estimates using an Euler integration
	gyro_est.roll = gyro_est.roll + delta_t * gyro_rates.roll;
	gyro_est.pitch = gyro_est.pitch + delta_t * gyro_rates.pitch;

	//Multiply by alpha constant and combine to get Euler angles in degrees
	EulerAngles->roll = ((acc_est.roll)*ALPHA + (gyro_est.roll)*(1.0f - ALPHA)) * RAD_TO_DEG;
	EulerAngles->pitch = ((acc_est.pitch)*ALPHA + (gyro_est.pitch)*(1.0f - ALPHA)) * RAD_TO_DEG;

}
