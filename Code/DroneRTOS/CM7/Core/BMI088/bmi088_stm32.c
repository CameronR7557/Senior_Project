/**\mainpage
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file        bmi088.c
 * @date        24 Aug 2018
 * @version     1.2.0
 *
 */

/*! \file bmi088_stm32.c
 \STM32 specific SPI functions */
/****************************************************************************/
/**\name        Header files
 ****************************************************************************/
#include "bmi08x.h"
#include "bmi088.h"
#include "bmi088_stm32.h"
#include "stm32h7xx_hal.h"

extern uint8_t bmi_init;
extern SPI_HandleTypeDef hspi6;
extern I2C_HandleTypeDef hi2c2;

#define BMI088_SPI &hspi6
#define BMI088_I2C &hi2c2



int8_t rslt;
uint32_t error;
struct bmi08x_sensor_data_f user_accel_bmi088;
struct bmi08x_sensor_data_f user_gyro_bmi088;

//struct bmi08x_dev dev = {
//        .accel_id = CS_ACC_Pin,
//        .gyro_id = CS_GYRO_Pin,
//        .intf = BMI08X_SPI_INTF,
//        .read = &stm32_spi_read,//user_spi_read
//        .write = &stm32_spi_write,//user_spi_write
//        .delay_ms = &HAL_Delay//user_delay_milli_sec ----- May need to change this to osDelay
//		//.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ,
//		//.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL,
//		//.accel_cfg.range = BMI088_ACCEL_RANGE_3G
//};

struct bmi08x_dev dev = {
        .accel_id = (BMI08X_ACCEL_I2C_ADDR_SECONDARY << 1) ,
        .gyro_id = (BMI08X_GYRO_I2C_ADDR_SECONDARY << 1),
        .intf = BMI08X_I2C_INTF,
        .read = &stm32_i2c_read,//user_spi_read
        .write = &stm32_i2c_write,//user_spi_write
        .delay_ms = &HAL_Delay//try changing to osDelay when inside a task using IMU functions
		//.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ,
		//.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL,
		//.accel_cfg.range = BMI088_ACCEL_RANGE_3G
};

//These may not need to be global. May be able to stick in BMI init func below
struct bmi08x_int_cfg int_config;
struct bmi08x_data_sync_cfg sync_cfg;

int8_t stm32_spi_write(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	reg_addr &= 0x7f;
	HAL_GPIO_WritePin(GPIOE, cs_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(BMI088_SPI, &reg_addr, 1, HAL_MAX_DELAY);
	while(HAL_SPI_GetState(BMI088_SPI) == HAL_SPI_STATE_BUSY);
	HAL_SPI_Transmit(BMI088_SPI, data, len, HAL_MAX_DELAY);
	while(HAL_SPI_GetState(BMI088_SPI) == HAL_SPI_STATE_BUSY);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOE, cs_pin, GPIO_PIN_SET);
	return 0;
}

int8_t stm32_spi_read(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	reg_addr |= 0x80;
	HAL_GPIO_WritePin(GPIOE, cs_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(BMI088_SPI, &reg_addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(BMI088_SPI, data, len, HAL_MAX_DELAY);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOE, cs_pin, GPIO_PIN_SET);
	return 0;
}


//#define BMI08X_ACCEL_I2C_ADDR_PRIMARY               UINT8_C(0x18)
//#define BMI08X_ACCEL_I2C_ADDR_SECONDARY             UINT8_C(0x19)
//#define BMI08X_GYRO_I2C_ADDR_PRIMARY                UINT8_C(0x68)
//#define BMI08X_GYRO_I2C_ADDR_SECONDARY              UINT8_C(0x69)
int8_t stm32_i2c_write(uint8_t address, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	HAL_I2C_Mem_Write(&hi2c2, address, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
	while(HAL_I2C_GetState(BMI088_I2C) == HAL_I2C_STATE_BUSY);
	return 0;
}

int8_t stm32_i2c_read(uint8_t address, uint8_t reg_addr, uint8_t *data, uint16_t len)
{

	HAL_I2C_Mem_Read(&hi2c2, address, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
	while(HAL_I2C_GetState(BMI088_I2C) == HAL_I2C_STATE_BUSY);
	return 0;
}


void BMI088_Init()
{
	//error = HAL_I2C_IsDeviceReady(&hi2c2, (BMI08X_ACCEL_I2C_ADDR_SECONDARY << 1), 100, HAL_MAX_DELAY);
	//error = HAL_I2C_IsDeviceReady(&hi2c2, (BMI08X_GYRO_I2C_ADDR_SECONDARY << 1), 100, HAL_MAX_DELAY);

	/* Initialize bmi085 sensors (accel & gyro)*/
	rslt = bmi088_init(&dev);

	/* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */
	rslt = bmi08a_soft_reset(&dev);

	/*! Max read/write length (maximum supported length is 32).
	 To be set by the user */
	dev.read_write_len = 32;
	/*set accel power mode */
	dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	rslt = bmi08a_set_power_mode(&dev);

	dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	bmi08g_set_power_mode(&dev);

	/* API uploads the bmi08x config file onto the device and wait for 150ms
	 *      to enable the data synchronization - delay taken care inside the function */
	rslt = bmi088_apply_config_file(&dev);

	/*assign accel range setting*/
	dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;//BMI088_ACCEL_RANGE_3G;//Be sure to change conv consts if you change ranges
	/*assign gyro range setting*/
	//dev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
	dev.gyro_cfg.range =  BMI08X_GYRO_RANGE_2000_DPS;//BMI08X_GYRO_RANGE_1000_DPS;//Be sure to change conv consts if you change ranges
	/*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
	//sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ;

	sync_cfg.mode = 3;
	rslt = bmi088_configure_data_synchronization(sync_cfg, &dev);

	/*uint8_t data;
	data = 0x8A;
	bmi08a_set_regs(BMI08X_ACCEL_CONF_REG, &data, 1, &dev);
	data = 0x07;
	bmi08g_set_regs(BMI08X_GYRO_BANDWIDTH_REG, &data, 1, &dev);*/

	int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
	int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
	int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
	int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
	int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

	int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
	int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
	int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

	int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
	int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
	int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

	/* Enable synchronization interrupt pin */
	rslt = bmi088_set_data_sync_int_config(&int_config, &dev);

	//rslt = bmi08a_perform_selftest(&dev);
	//rslt = bmi08g_perform_selftest(&dev);

	bmi_init = 1;


}

/** @}*/
