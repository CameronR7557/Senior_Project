

#include "icp_10111.h"
#include "math.h"

void ICP_OTP_Read(inv_invpres_t* icp_data)
{
	uint8_t pressureSensorTX[5] = {0xC5, 0x95, 0x00, 0x66, 0x9C};
	uint8_t pressureSensorRX[3] = {};
	HAL_I2C_Master_Transmit(&hi2c1, ICP_ADDR, pressureSensorTX, 5, 1000);//Set up OTP read
	HAL_Delay(24);								   //(Check status here instead)
	pressureSensorTX[0] = 0xC7;
	pressureSensorTX[1] = 0xF7;
	for(int i = 0; i < 4; ++i)
	{
		HAL_I2C_Master_Transmit(&hi2c1, ICP_ADDR, pressureSensorTX, 2, 1000);//Get OTP Value
		HAL_Delay(24);//Check status instead
		HAL_I2C_Master_Receive(&hi2c1, ICP_ADDR, pressureSensorRX, 3, 1000);//Read OTP Value
		HAL_Delay(24);//Check status instead
		//Can check CRC here
		icp_data->sensor_constants[i] = (float)(pressureSensorRX[0] << 8 | pressureSensorRX[1]); //Store OTP Value
	}
}

void ICP_Init(inv_invpres_t* icp_data)
{
	//Do I know where these values come from? No
	icp_data->p_Pa_calib[0] = 45000.0;
	icp_data->p_Pa_calib[1] = 80000.0;
	icp_data->p_Pa_calib[2] = 105000.0;
	icp_data->LUT_lower = 3.5 * (1<<20);
	icp_data->LUT_upper = 11.5 * (1<<20);
	icp_data->quadr_factor = 1 / 16777216.0;
	icp_data->offst_factor = 2048.0;

	ICP_OTP_Read(icp_data);

}

// p_Pa -- List of 3 values corresponding to applied pressure in Pa
// p_LUT -- List of 3 values corresponding to the measured p_LUT values at the applied pressures.
void calculate_conversion_constants(inv_invpres_t * s, float *p_Pa, float *p_LUT, float *out)
{
	float A,B,C;
	C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
		p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
		p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
		(p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
		p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
		p_LUT[1] * (p_Pa[2] - p_Pa[0]));
	A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);
	B = (p_Pa[0] - A) * (p_LUT[0] + C);
	out[0] = A;
	out[1] = B;
	out[2] = C;
}

// p_LSB -- Raw pressure data from sensor
// T_LSB -- Raw temperature data from sensor
int inv_invpres_process_data(inv_invpres_t * s, int p_LSB, int T_LSB, float * height, float* pressure)
{
	double gravity = -9.80665; //Acc due to gravity
	double M = 0.0289644;     //Molar mass of air
	double R = 8.31432;       //Universal gas constant
	float temperature = 0.0;
	float t;
	float s1,s2,s3;
	float in[3];
	float out[3];
	float A,B,C;

	t = (float)(T_LSB - 32768);
	s1 = s->LUT_lower + (float)(s->sensor_constants[0] * t * t) * s->quadr_factor;
	s2 = s->offst_factor * s->sensor_constants[3] + (float)(s->sensor_constants[1] * t * t) * s->quadr_factor;
	s3 = s->LUT_upper + (float)(s->sensor_constants[2] * t * t) * s->quadr_factor;
	in[0] = s1;
	in[1] = s2;
	in[2] = s3;
	calculate_conversion_constants(s, s->p_Pa_calib, in, out);
	A = out[0];
	B = out[1];
	C = out[2];
	*pressure = A + B / (C + p_LSB);
	temperature = (-45.f + 175.f/65536.f * T_LSB) + 273.15;//Temp in Kelvin
	if(height != NULL)
		*height = (log((*pressure / s->P0)))*(R*temperature)/(gravity*M);

	return 0;
}

