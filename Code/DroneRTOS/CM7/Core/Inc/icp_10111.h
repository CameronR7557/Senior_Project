#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"

#define ICP_ADDR (0x63 << 1)

typedef struct
{
	uint32_t min_delay_us;
	uint8_t pressure_en;
	uint8_t temperature_en;
	float sensor_constants[4]; // OTP values
	float p_Pa_calib[3];
	float LUT_lower;
	float LUT_upper;
	float quadr_factor;
	float offst_factor;
	float P0;
} inv_invpres_t;

void ICP_OTP_Read();
void ICP_Init(inv_invpres_t* icp_data);
void calculate_conversion_constants(inv_invpres_t * s, float *p_Pa, float *p_LUT, float *out);
int inv_invpres_process_data(inv_invpres_t * s, int p_LSB, int T_LSB, float * height, float* pressure);
