/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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


//Transfer learning and machine reinforcement learning for drone.
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01p.h"
#include "dshot.h"
#include "bmi088_stm32.h"
#include "bmi08x.h"
#include "bmi088.h"
#include "../DebugRedirect/retarget.h"
#include "../FlightController/flightcontroller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define JS_DEADZONE 40

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RadioTX[32] = {0};
uint8_t RadioRX[32] = {0};
uint8_t bmi_init = 0;
uint8_t lidarPacket[47] = {0};
uint8_t rangeFinderPacket[24] = {0};
uint8_t firstMotorValsFlag = 1;
//uint16_t motors[4] = {0,0,0,0};

extern osMessageQueueId_t DShotQueueHandle;
extern osMessageQueueId_t motorControlQueueHandle;
extern osMessageQueueId_t lidarQueueHandle;
extern osMessageQueueId_t rangeFinderQueueHandle;
//extern motor_values;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  //Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
//Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ETH_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  MX_SPI6_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart3);
  nrf24l01p_tx_init(2500, _2Mbps);//May want to change to 2Mbps
  HAL_UART_Receive_DMA(&huart1, lidarPacket, 47);
  dshot_init(DSHOT600);
  BMI088_Init();

  HAL_Delay(100); //Delay for radio and any other initialization

  HAL_GPIO_WritePin(LD19_PWR_GPIO_Port, LD19_PWR_Pin, GPIO_PIN_SET);//Turn on LD19 LiDAR
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 10;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6|RCC_PERIPHCLK_SPI3
                              |RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 75;
  PeriphClkInitStruct.PLL2.PLL2P = 12;
  PeriphClkInitStruct.PLL2.PLL2Q = 60;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	motor_values motor_vals = {0,0,0,0};
	uint8_t status = 0;
	//TX
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
	{
		//uint8_t addrs[5] = {};
		status = nrf24l01p_get_status();
		if((status & 0b00010000))//Check MAX_RT interrupt. Need to clear to enable comm.
		{
			nrf24l01p_clear_max_rt();

			if(htim7.Instance->CNT >= 3999)//Reset
			{
				//If radio disconnects, turn off motors -- Can be removed later, for flight testing low to gnd
				motor_vals.pitch = 0;
				motor_vals.roll = 0;
				motor_vals.yaw = 0;
				motor_vals.throttle = 0;
				osMessageQueuePut(motorControlQueueHandle, &motor_vals, 0U, 0U);

				nrf24l01p_clear_max_rt();
				nrf24l01p_reset_tx_addrs();
				nrf24l01p_flush_rx_fifo();
				nrf24l01p_flush_tx_fifo();
				//nrf24l01p_power_down();
				//nrf24l01p_power_up();
				nrf24l01p_tx_init(2500, _2Mbps);//Change init settings to match if they are ever changed
				osDelay(100);
				htim7.Instance->CNT = 0;
			}
		}
		else if(status & 0b01000000)//Check if RX_DR status bit is set. If it is set, there was an ack w/ payload. RX_DR and TX_DS should both be asserted in this case.
		{
			htim7.Instance->CNT = 0;
			nrf24l01p_read_rx_fifo(RadioRX, 16);//need to make sure payload length define is set correctly
			nrf24l01p_clear_rx_dr(); //Clear TX_DR interrupt flag

			//Force Throttle value to 0 or 47 depending on what ESC wants for 0 throttle
			motor_vals.pitch = (int16_t)(((RadioRX[0] << 8) | RadioRX[1]) - 1024);
			motor_vals.roll = (int16_t)(((RadioRX[2] << 8) | RadioRX[3]) - 1024);
			motor_vals.yaw = (int16_t)(((RadioRX[4] << 8) | RadioRX[5]) - 1024);
			motor_vals.throttle = ((RadioRX[6] << 8) | RadioRX[7]);

			//Only claculate and update motor controls if the values are reasonable. For some reason, the radio will receive very high values. May be an issue with the radio. Not sure.
			if(motor_vals.throttle < 2200 && abs(motor_vals.yaw) < 1200 && abs(motor_vals.pitch) < 1200 && abs(motor_vals.roll) < 1200)
			{
				//Deadzones
				if(motor_vals.throttle < (48 + JS_DEADZONE))
					motor_vals.throttle = 0;
				else if(motor_vals.throttle > MAX_THROTTLE)
					motor_vals.throttle = MAX_THROTTLE;

				if(motor_vals.yaw > -JS_DEADZONE && motor_vals.yaw < JS_DEADZONE)
					motor_vals.yaw = 0;

				if(motor_vals.roll > -JS_DEADZONE && motor_vals.roll < JS_DEADZONE)
					motor_vals.roll = 0;

				if(motor_vals.pitch > -JS_DEADZONE && motor_vals.pitch < JS_DEADZONE)
					motor_vals.pitch = 0;

				osMessageQueuePut(motorControlQueueHandle, &motor_vals, 0U, 0U);
			}
		}
		nrf24l01p_tx_irq(); // clear interrupt flag (TX_DS)

	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		//HAL_UART_Receive_DMA(&huart1, lidarPacket, 47);
		//uint8_t dataValid = 1;
		osMessageQueuePut(lidarQueueHandle, lidarPacket, 0U, 0U);
	}
	else if(huart == &huart5)
	{
		if(rangeFinderPacket[1] == 0xE0)
			osMessageQueuePut(rangeFinderQueueHandle, rangeFinderPacket, 0U, 0U);
		//HAL_UART_Receive_DMA(&huart5, buffer, 24);
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
