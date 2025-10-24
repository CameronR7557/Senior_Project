/*! \file bmi088_stm32.h
 \STM32 specific SPI functions */

/*********************************************************************/
/* header files */
#include "bmi08x_defs.h"
#include "main.h"
#ifndef BMI_STM
#define BMI_STM

int8_t stm32_spi_write(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t stm32_spi_read(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t stm32_i2c_write(uint8_t address, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t stm32_i2c_read(uint8_t address, uint8_t reg_addr, uint8_t *data, uint16_t len);

void BMI088_Init();

extern struct bmi08x_sensor_data_f user_accel_bmi088;
extern struct bmi08x_sensor_data_f user_gyro_bmi088;
extern struct bmi08x_dev dev;

#endif
