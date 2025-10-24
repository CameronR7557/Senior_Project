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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341_gfx.h"
#include "ili9341_font.h"
#include "ili9341.h"
#include "nrf24l01p.h"
#include "FIRFilter.h"
#include "../DebugRedirect/retarget.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define NUM_SUMS	5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern osMessageQueueId_t radioRXQueueHandle;
uint8_t ADC1seq = 0; //Number to keep track of which ADC1 channel should be converted
uint8_t ADC2seq = 0; //Track ADC2 conversions

uint16_t throttle_range[2] = {4179, 60996};
uint16_t yaw_range[3] = {5177, 32950, 62918};//Disregard middle values, they are not averaged with much data
uint16_t pitch_range[3] = {3621, 39000, 59969};
uint16_t roll_range[3] = {4949, 33000, 62432};

uint16_t cur_throttle = 0;
uint16_t cur_yaw = 0;
uint16_t cur_pitch = 0;
uint16_t cur_roll = 0;
//uint8_t first_val = 1; //Flag for determining when to reset joystick running average

FIRFilter throttle_filter;
FIRFilter yaw_filter;
FIRFilter pitch_filter;
FIRFilter roll_filter;

int sums[4] = {0};
int throttleVals[NUM_SUMS] = {0};
int yawVals[NUM_SUMS] = {0};
int pitchVals[NUM_SUMS] = {0};
int rollVals[NUM_SUMS] = {0};
uint8_t sums_index = 0;
uint8_t firstRadioRX = 1;

uint8_t RadioTX[32] = {0};
uint8_t RadioRX[16] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC2_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart3);
//  FIRFilter_Init(&throttle_filter);
//  FIRFilter_Init(&yaw_filter);
//  FIRFilter_Init(&pitch_filter);
//  FIRFilter_Init(&roll_filter);
  nrf24l01p_rx_init(2500, _2Mbps);//May want to change to 2Mbps
  HAL_TIM_Base_Start_IT(&htim3);//Start timer for ADC
  nrf24l01p_write_ack_payload(RadioTX, 29);
  HAL_GPIO_WritePin(GPIOG, Red_LED_Pin, 1);//Turn on PWR LED


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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 75;
  PeriphClkInitStruct.PLL2.PLL2P = 12;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 1;
  PeriphClkInitStruct.PLL3.PLL3N = 100;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 20;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//uint8_t addrs[5] = {};	//Testing
	static uint16_t radioCount = 0;
	uint8_t status;
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
	{
		if(firstRadioRX)
		{
			  firstRadioRX = 0;
			  HAL_TIM_Base_Start(&htim7);   //Start timer for Radio Reconnect
			  htim7.Instance->CNT = 0;
		}

		//status = nrf24l01p_get_status();
		radioCount++;
		if(radioCount >= 300)//Toggle LED every 300 TX/RX cycles
		{
			HAL_GPIO_TogglePin(GPIOG, Green_LED_Pin);//Toggle Radio LED
			radioCount = 0;
		}
		nrf24l01p_rx_receive(RadioRX, 16); // read data when data ready flag is set
		nrf24l01p_write_ack_payload(RadioTX, 16);
		osMessageQueuePut(radioRXQueueHandle, RadioRX, 0U, 0U);//Send received data to display task queue

		htim7.Instance->CNT = 0;
	}
}

void HAL_DMA_TX_CpltCallback(DMA_HandleTypeDef* hdma)//Not really being used as far as I remember. For ILI
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

//****************************************************
//Purpose: Handle ADC conversion completions. Store
//		   ADC data.
//****************************************************
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint32_t adc_val = 0;
	if(hadc == &hadc1)
	{
		//Pitch
		adc_val = HAL_ADC_GetValue(&hadc1);
		adc_val = round((2047*(abs(adc_val - pitch_range[0])))/(pitch_range[2] - pitch_range[0]));
		sums[2] = sums[2] - pitchVals[sums_index];//HERE
		sums[2] = sums[2] + adc_val;
		pitchVals[sums_index] = adc_val;
		cur_pitch = sums[2] / NUM_SUMS;
		RadioTX[1] = cur_pitch;
		RadioTX[0] = (cur_pitch >> 8);//MSBs

		//Roll
		adc_val = HAL_ADC_GetValue(&hadc1);
		adc_val = round((2047*(abs(adc_val - roll_range[0])))/(roll_range[2] - roll_range[0]));
		sums[3] = sums[3] - rollVals[sums_index];//HERE
		sums[3] = sums[3] + adc_val;
		rollVals[sums_index] = adc_val;
		cur_roll = sums[3] / NUM_SUMS;
		RadioTX[3] = cur_roll;
		RadioTX[2] = (cur_roll >> 8);//MSBs
		ADC1seq = 1;
		HAL_ADC_Stop_IT(&hadc1);
	}
	else//hadc2
	{
		//Yaw
		adc_val = HAL_ADC_GetValue(&hadc2);
		adc_val = round((2047*(abs(adc_val - yaw_range[0])))/(yaw_range[2] - yaw_range[0]));
		sums[1] = sums[1] - yawVals[sums_index];//HERE
		sums[1] = sums[1] + adc_val;
		yawVals[sums_index] = adc_val;
		cur_yaw = sums[1] / NUM_SUMS;
		RadioTX[5] = cur_yaw;
		RadioTX[4] = (cur_yaw >> 8);//MSBs

		//Throttle
		adc_val = HAL_ADC_GetValue(&hadc2);
		adc_val = round((2047*(abs(adc_val - throttle_range[0])))/(throttle_range[1] - throttle_range[0]));
		sums[0] = sums[0] - throttleVals[sums_index];//HERE
		sums[0] = sums[0] + adc_val;
		throttleVals[sums_index] = adc_val;
		cur_throttle = sums[0] / NUM_SUMS;
		RadioTX[7] = cur_throttle;//LSBs -- Throttle
		RadioTX[6] = ( cur_throttle >> 8);//MSBs
		ADC2seq = 1;
		HAL_ADC_Stop_IT(&hadc2);
	}
	//If both adcs are done with both conversion, re-enable timer interrupt to begin again
	if(ADC1seq == 1 && ADC2seq == 1)
	{
		sums_index = (sums_index + 1) % NUM_SUMS;//Increment index
		//restart timer interrupt
		//htim3.Instance->CNT = 0;//Should set timer count to 0
		printf("%d, %d, %d, %d\r\n", cur_throttle, cur_yaw, cur_pitch, cur_roll);
		HAL_TIM_Base_Start_IT(&htim3);
		ADC1seq = 0;
		ADC2seq = 0;
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
  if (htim == &htim3)
  {
	  //Start conversions for adc1 and adc2
	  HAL_ADC_Start_IT(&hadc1);
	  HAL_ADC_Start_IT(&hadc2);
	  //Stop timer interrupt for tim3 (re-enable in ADC int)
	  HAL_TIM_Base_Stop_IT(&htim3);
  }
  else if(htim == &htim4)//Make sure this is working (nrf reset if no rx in .125 sec)
  {
//	  nrf24l01p_rx_init(2500, _1Mbps);//May want to change to 2Mbps
//	  nrf24l01p_write_ack_payload(RadioTX, 8);
  }
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
