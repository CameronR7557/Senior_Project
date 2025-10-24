/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341_gfx.h"
#include "ili9341_font.h"
#include "ili9341.h"
#include "nrf24l01p.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_MOTOR_VALS 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;
/* USER CODE END Variables */
/* Definitions for blinkTask */
osThreadId_t blinkTaskHandle;
const osThreadAttr_t blinkTask_attributes = {
  .name = "blinkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for radioTask */
osThreadId_t radioTaskHandle;
const osThreadAttr_t radioTask_attributes = {
  .name = "radioTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LCDtask */
osThreadId_t LCDtaskHandle;
const osThreadAttr_t LCDtask_attributes = {
  .name = "LCDtask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for radioRXQueue */
osMessageQueueId_t radioRXQueueHandle;
const osMessageQueueAttr_t radioRXQueue_attributes = {
  .name = "radioRXQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void RadioTX_RX(void *argument);
void LCD_Display(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of radioRXQueue */
  radioRXQueueHandle = osMessageQueueNew (5, 16, &radioRXQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blinkTask */
  blinkTaskHandle = osThreadNew(StartDefaultTask, NULL, &blinkTask_attributes);

  /* creation of radioTask */
  radioTaskHandle = osThreadNew(RadioTX_RX, NULL, &radioTask_attributes);

  /* creation of LCDtask */
  LCDtaskHandle = osThreadNew(LCD_Display, NULL, &LCDtask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the blinkTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);//Toggle LED
		osDelay(1000U);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_RadioTX_RX */
/**
* @brief Function implementing the radioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RadioTX_RX */
void RadioTX_RX(void *argument)
{
  /* USER CODE BEGIN RadioTX_RX */
	int16_t delay = 3000;
	const uint16_t RECOVERY_DELAY = 3999;//400ms. Counter is 0.1 ms every tick
  /* Infinite loop */
  while(1)
  {
//	if(htim7.Instance->CNT >= RECOVERY_DELAY)//400ms
//	{
//		nrf24l01p_clear_max_rt();
//		nrf24l01p_reset_tx_addrs();
//		nrf24l01p_flush_rx_fifo();
//		nrf24l01p_flush_tx_fifo();
//		//nrf24l01p_power_down();
//		//nrf24l01p_power_up();
//		nrf24l01p_tx_init(2500, _2Mbps);//Change init settings to match if they are ever changed
//		htim7.Instance->CNT = 0;
//		delay = 300;
//	}
//	else
//	{
//		delay = (RECOVERY_DELAY - htim7.Instance->CNT) / 10;
//	}

	if(delay < 10)
		delay = 10;

    osDelay(1000);
  }
  /* USER CODE END RadioTX_RX */
}

/* USER CODE BEGIN Header_LCD_Display */
/**
* @brief Function implementing the LCDtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCD_Display */
void LCD_Display(void *argument)
{
  /* USER CODE BEGIN LCD_Display */
	HAL_NVIC_DisableIRQ(ADC_IRQn);
	//__disable_irq();
	struct ili9341_t* LCD_Screen = ili9341_new(
		&hspi1,
		LCD_RESET_GPIO_Port, LCD_RESET_Pin,
		LCD_CS_GPIO_Port, LCD_CS_Pin,
		LCD_DC_GPIO_Port, LCD_DC_Pin,
		3,
		NULL, 0,
		NULL, 0,
		0,
		-1
	);

	//int count = 0;

	uint8_t packet[16] = {0};
	//uint16_t lidarData[14] = {0};
	uint16_t beginAngle = 0;
	uint16_t endAngle = 0;
	uint16_t packetCount;
	ili9341_text_attr_t text1 = {&ili9341_font_11x18, ILI9341_WHITE, ILI9341_BLACK, 1, 1};
	ili9341_fill_screen(LCD_Screen, ILI9341_WHITE);
	ili9341_fill_rect(LCD_Screen, ILI9341_WHITE, 0, 0, 320, 120);
	ili9341_fill_rect(LCD_Screen, ILI9341_WHITE, 0, 120, 320, 120);
	ili9341_fill_rect(LCD_Screen, ILI9341_WHITE, 0, 190, 320, 50);
	ili9341_fill_rect(LCD_Screen, ILI9341_BLUE, 199, 119, 3, 3);
	ili9341_fill_rect(LCD_Screen, ILI9341_BLACK, 75, 0, 4, 240);
	osMessageQueueReset(radioRXQueueHandle);
	//HAL_TIM_Base_Start_IT(&htim13);//For LCD frame rate timing
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	//__enable_irq();
  /* Infinite loop */
	while(1)
	{
		packetCount = osMessageQueueGetCount(radioRXQueueHandle);
		if(packetCount)//Receives faster than it is able to display
		{
			osMessageQueueGet(radioRXQueueHandle, packet, 0U, 0U);

			if(packet[0] != 0xff && packet[1] != 0xff)//Get LiDAR data from packet
			{
				beginAngle = ((uint16_t)packet[1] << 8) | (uint16_t)packet[0];
				endAngle = ((uint16_t)packet[3] << 8) | (uint16_t)packet[2];

				displayData(LCD_Screen, beginAngle, endAngle, &packet[4], 12);

			}
//			count++;
//			if(htim13.Instance->CNT > 9999)
//			{
//				printf("LCD Updates/Second: %f\r\n", (float)((float)count)/((float)htim13.Instance->CNT/10000.0f));
//				count = 0;
//				htim13.Instance->CNT = 0;
//			}
			osDelay(1);
		}
	}
  /* USER CODE END LCD_Display */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

