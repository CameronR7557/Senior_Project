/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_CSN_Pin GPIO_PIN_3
#define SPI2_CSN_GPIO_Port GPIOE
#define CS_GYRO_Pin GPIO_PIN_4
#define CS_GYRO_GPIO_Port GPIOE
#define CS_ACC_Pin GPIO_PIN_6
#define CS_ACC_GPIO_Port GPIOE
#define LD19_PWR_Pin GPIO_PIN_4
#define LD19_PWR_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define RF_RX_Pin GPIO_PIN_12
#define RF_RX_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define CE_NRF_Pin GPIO_PIN_11
#define CE_NRF_GPIO_Port GPIOD
#define NRF_INT_Pin GPIO_PIN_6
#define NRF_INT_GPIO_Port GPIOG
#define NRF_INT_EXTI_IRQn EXTI9_5_IRQn
#define GYRO_INT_Pin GPIO_PIN_7
#define GYRO_INT_GPIO_Port GPIOC
#define GYRO_INT_EXTI_IRQn EXTI9_5_IRQn
#define ACC_INT_Pin GPIO_PIN_8
#define ACC_INT_GPIO_Port GPIOC
#define ACC_INT_EXTI_IRQn EXTI9_5_IRQn
#define RF_TX_Pin GPIO_PIN_12
#define RF_TX_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
