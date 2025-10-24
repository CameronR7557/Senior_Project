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
#include "nrf24l01p.h"
#include "dshot.h"
#include "usart.h"
#include "i2c.h"
#include "ld19_lidar.h"
#include "icp_10111.h"
#include "lidar07_rangefinder.h"
#include "bmi088_stm32.h"
#include "bmi08x.h"
#include "../FlightController/flightcontroller.h"
#include "../FlightController/collisionAvoidance.h"
#include <stdio.h>
#include "../Filters/ExtendedKalmanFilter.h"
#include "../Filters/KalmanRollPitchYaw.h"
#include "../Filters/LowPassFilter.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Biases found by setting drone on flat surface and recording then averaging readings
#define GYRO_X_BIAS	0.000058235f
#define GYRO_Y_BIAS	0.008710119f
#define GYRO_Z_BIAS	0.000948187f
#define ACC_X_BIAS	-0.273271355f
#define ACC_Y_BIAS	-0.143867801f
#define ACC_Z_BIAS	9.617319141f	//Dont want to subtract, gravity is accounted for already

#define SEND_MOTOR_VALS	0

#define DELAY_US ( ( TickType_t ) 1000000 / configTICK_RATE_HZ )

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t RadioTX[32];
extern uint8_t RadioRX[32];
extern uint8_t rangeFinderPacket[24];
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
//extern uint8_t lidarPacket[47];
inv_invpres_t icp_data;
float yawEst = 0.0f;
DroneRotations globalRotationVals = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* USER CODE END Variables */
/* Definitions for blinkTask */
osThreadId_t blinkTaskHandle;
const osThreadAttr_t blinkTask_attributes = {
  .name = "blinkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for taskRadioTX */
osThreadId_t taskRadioTXHandle;
const osThreadAttr_t taskRadioTX_attributes = {
  .name = "taskRadioTX",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for taskMotorCtrl */
osThreadId_t taskMotorCtrlHandle;
const osThreadAttr_t taskMotorCtrl_attributes = {
  .name = "taskMotorCtrl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskLidarRX */
osThreadId_t taskLidarRXHandle;
const osThreadAttr_t taskLidarRX_attributes = {
  .name = "taskLidarRX",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskPressureSen */
osThreadId_t taskPressureSenHandle;
const osThreadAttr_t taskPressureSen_attributes = {
  .name = "taskPressureSen",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskRangeFinder */
osThreadId_t taskRangeFinderHandle;
const osThreadAttr_t taskRangeFinder_attributes = {
  .name = "taskRangeFinder",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskReadIMU */
osThreadId_t taskReadIMUHandle;
const osThreadAttr_t taskReadIMU_attributes = {
  .name = "taskReadIMU",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UpdatePIDTask */
osThreadId_t UpdatePIDTaskHandle;
const osThreadAttr_t UpdatePIDTask_attributes = {
  .name = "UpdatePIDTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CA_Task */
osThreadId_t CA_TaskHandle;
const osThreadAttr_t CA_Task_attributes = {
  .name = "CA_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorControlQueue */
osMessageQueueId_t motorControlQueueHandle;
const osMessageQueueAttr_t motorControlQueue_attributes = {
  .name = "motorControlQueue"
};
/* Definitions for lidarQueue */
osMessageQueueId_t lidarQueueHandle;
const osMessageQueueAttr_t lidarQueue_attributes = {
  .name = "lidarQueue"
};
/* Definitions for DShotQueue */
osMessageQueueId_t DShotQueueHandle;
const osMessageQueueAttr_t DShotQueue_attributes = {
  .name = "DShotQueue"
};
/* Definitions for rotationsQueue */
osMessageQueueId_t rotationsQueueHandle;
const osMessageQueueAttr_t rotationsQueue_attributes = {
  .name = "rotationsQueue"
};
/* Definitions for rangeFinderQueue */
osMessageQueueId_t rangeFinderQueueHandle;
const osMessageQueueAttr_t rangeFinderQueue_attributes = {
  .name = "rangeFinderQueue"
};
/* Definitions for pressureSensorQueue */
osMessageQueueId_t pressureSensorQueueHandle;
const osMessageQueueAttr_t pressureSensorQueue_attributes = {
  .name = "pressureSensorQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void blinkTaskFunc(void *argument);
void taskRadioTxFunc(void *argument);
void MotorControl(void *argument);
void lidarRX(void *argument);
void readPressureSensor(void *argument);
void measureHeight(void *argument);
void readIMU(void *argument);
void taskUpdatePIDs(void *argument);
void collisionAvoidance(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  //Get initial pressure value for calculating altitude. Doing it here before any other tasks are running.
  uint8_t pressureSensorTX[2] = {0x50, 0x59};
  uint8_t pressureSensorRX[9] = {};			//(Pressure MMSB, MLSB, Checksum, LMSB, LLSB, Checksum, Temp MSB, LSB, Checksum)
  uint32_t rawPressureData = 0;
  uint32_t rawTempData = 0;
  ICP_Init(&icp_data);
  HAL_I2C_Master_Transmit(&hi2c1, ICP_ADDR, pressureSensorTX, 2, 1000);
  HAL_Delay(24);
  HAL_I2C_Master_Receive(&hi2c1, ICP_ADDR, pressureSensorRX, 9, 1000);
  rawPressureData = pressureSensorRX[0] << 16 | pressureSensorRX[1] << 8 | pressureSensorRX[3];
  rawTempData = pressureSensorRX[6] << 8 | pressureSensorRX[7];
  inv_invpres_process_data(&icp_data, rawPressureData , rawTempData, NULL, &(icp_data.P0));
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
  /* creation of motorControlQueue */
  motorControlQueueHandle = osMessageQueueNew (1, 8, &motorControlQueue_attributes);

  /* creation of lidarQueue */
  lidarQueueHandle = osMessageQueueNew (10, 47, &lidarQueue_attributes);

  /* creation of DShotQueue */
  DShotQueueHandle = osMessageQueueNew (1, 8, &DShotQueue_attributes);

  /* creation of rotationsQueue */
  rotationsQueueHandle = osMessageQueueNew (1, 24, &rotationsQueue_attributes);

  /* creation of rangeFinderQueue */
  rangeFinderQueueHandle = osMessageQueueNew (1, 24, &rangeFinderQueue_attributes);

  /* creation of pressureSensorQueue */
  pressureSensorQueueHandle = osMessageQueueNew (1, sizeof(float), &pressureSensorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blinkTask */
  blinkTaskHandle = osThreadNew(blinkTaskFunc, NULL, &blinkTask_attributes);

  /* creation of taskRadioTX */
  taskRadioTXHandle = osThreadNew(taskRadioTxFunc, NULL, &taskRadioTX_attributes);

  /* creation of taskMotorCtrl */
  taskMotorCtrlHandle = osThreadNew(MotorControl, NULL, &taskMotorCtrl_attributes);

  /* creation of taskLidarRX */
  taskLidarRXHandle = osThreadNew(lidarRX, NULL, &taskLidarRX_attributes);

  /* creation of taskPressureSen */
  taskPressureSenHandle = osThreadNew(readPressureSensor, NULL, &taskPressureSen_attributes);

  /* creation of taskRangeFinder */
  taskRangeFinderHandle = osThreadNew(measureHeight, NULL, &taskRangeFinder_attributes);

  /* creation of taskReadIMU */
  taskReadIMUHandle = osThreadNew(readIMU, NULL, &taskReadIMU_attributes);

  /* creation of UpdatePIDTask */
  UpdatePIDTaskHandle = osThreadNew(taskUpdatePIDs, NULL, &UpdatePIDTask_attributes);

  /* creation of CA_Task */
  CA_TaskHandle = osThreadNew(collisionAvoidance, NULL, &CA_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_blinkTaskFunc */
/**
  * @brief  Function implementing the blinkTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_blinkTaskFunc */
void blinkTaskFunc(void *argument)
{
  /* USER CODE BEGIN blinkTaskFunc */
  /* Infinite loop */
  while(1)
  {
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);//Red LED
	  osDelay(1000);
  }
  /* USER CODE END blinkTaskFunc */
}

/* USER CODE BEGIN Header_taskRadioTxFunc */
/**
* @brief Function implementing the taskRadioTX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskRadioTxFunc */
void taskRadioTxFunc(void *argument)
{
  /* USER CODE BEGIN taskRadioTxFunc */
	motor_values motor_vals = {0,0,0,0};
  /* Infinite loop */
  while(1)
  {
	  //RadioTX: {startAngle, startAngle2, endAngle, endAngle2, 24 bytes of distance measurements, 4 bytes free}
	  //nrf24l01p_tx_transmit(RadioTX, 32);
	  nrf24l01p_tx_transmit(RadioTX, 16);
	  HAL_TIM_Base_Start(&htim7);   //Start timer for Radio Reconnect
	  if(htim7.Instance->CNT >= 20000)
	  {
		  //If radio disconnects, turn off motors -- Can be removed later, for flight testing low to gnd
		  motor_vals.pitch = 0;
		  motor_vals.roll = 0;
		  motor_vals.yaw = 0;
		  motor_vals.throttle = 0;
		  osMessageQueuePut(motorControlQueueHandle, &motor_vals, 0U, 0U);
		  nrf24l01p_tx_init(2500, _2Mbps);
		  osDelay(100);
		  htim7.Instance->CNT = 0;
	  }
	  osDelay(5);
  }
  /* USER CODE END taskRadioTxFunc */
}

/* USER CODE BEGIN Header_MotorControl */
/**
* @brief Function implementing the taskMotorContro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControl */
void MotorControl(void *argument)
{
  /* USER CODE BEGIN MotorControl */
	DShotVals motors = {0,0,0,0};
	uint16_t dshotVals[4] = {0,0,0,0};
  /* Infinite loop */
  while(1)
  {
	  if(osMessageQueueGetCount(DShotQueueHandle))
	  {
		  osMessageQueueGet(DShotQueueHandle, &motors, NULL, 10);

		  dshotVals[0] = motors.motor1;
		  dshotVals[1] = motors.motor2;
		  dshotVals[2] = motors.motor3;
		  dshotVals[3] = motors.motor4;
		  //printf("DSHOT: %d, %d, %d, %d\r\n", dshotVals[0], dshotVals[1], dshotVals[2], dshotVals[3]);
	  }
	  //dshot_write(dshotVals);
	  osDelay(30);
  }
  /* USER CODE END MotorControl */
}

/* USER CODE BEGIN Header_lidarRX */
/**
* @brief Function implementing the taskLidarRX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lidarRX */
void lidarRX(void *argument)
{
  /* USER CODE BEGIN lidarRX */
	/* The LiDAR data gives the start and end angle of a data packet, may want to send this over radio as well to use for diplaying.
	 * See wiki.youyeetoo.com/en/Lidar/D300 section 3.3 for data example.
	 * Clockwise is positive.
	 */
	//__disable_irq();
	uint8_t packet[47];
	uint8_t packet0[47] = {0};
	uint8_t packet1[47] = {0};
	uint8_t packet_next[47];
	uint8_t twoPackets = 0;
	uint16_t fullDataPacket[14] = {0};

	uint8_t queueCount = 0;
	uint8_t crc = 0;
	uint8_t* header = packet;
	uint8_t headerFlag = 1;
	uint8_t headerIndex = 0;
	uint8_t tailIndex = 0;
	uint8_t packetFlag = 0;
	uint8_t* curPacket = packet0;
	uint8_t* nextPacket = packet1;
	uint8_t packetIndex = 0;
	LiDARFrameTypeDef lidarFrame;
	uint16_t goodPackets;
	uint16_t badPackets;
	uint16_t totalPackets;
	//__enable_irq();
  /* Infinite loop */
  while(1)
  {
	  queueCount = osMessageQueueGetCount(lidarQueueHandle);
	  if(queueCount)
	  {
		  osMessageQueueGet(lidarQueueHandle, packet, 0U, 0U);

		  if(queueCount > 1)
		  {
			  osMessageQueueGet(lidarQueueHandle, packet_next, 0U, 0U);
			  twoPackets = 1;
			  //osMessageQueueReset(lidarQueueHandle);//May or may not help
		  }
		  else
		  {
			  twoPackets = 0;
		  }

		  if(headerFlag)
		  {

			  while(headerFlag && packetIndex < 47)
			  {
				  if(packet[packetIndex] == 0x54 && (packetIndex < 46 && packet[packetIndex + 1] == 0x2c))
				  {
					  headerFlag = 0;
					  headerIndex = packetIndex;
					  tailIndex = 47 - headerIndex;
					  header = &packet[headerIndex];
					  packetFlag = 0;
				  }
				  packetIndex++;
			  }
			  packetIndex = 0;

		  }

		  if(header == NULL || header[0] != 0x54)
		  {
			  headerFlag = 1;
			  packetIndex = 0;
		  }


		  if(twoPackets)//if two, discard current and make entire packet from more current info and the set next to hold the tail of 2nd
		  {

			  for(uint8_t i = 0; i < (47 - headerIndex); ++i)//Copy header of next packet (may want to do after processing cur packet)
			  {
				  curPacket[i] = packet[headerIndex + i];
			  }

			  for(uint8_t i = 0; i < headerIndex; ++i)//Copy tail of current packet
			  {
				  curPacket[tailIndex + i] = packet_next[i];
			  }

			  for(uint8_t i = 0; i < (47 - headerIndex); ++i)//Copy header of next packet (may want to do after processing cur packet)
			  {
				  nextPacket[i] = packet_next[headerIndex + i];
			  }
		  }
		  else
		  {
			  for(uint8_t i = 0; i < headerIndex; ++i)//Copy tail of current packet
			  {
				  curPacket[tailIndex + i] = packet[i];
			  }

			  for(uint8_t i = 0; i < (47 - headerIndex); ++i)//Copy header of next packet
			  {
				  nextPacket[i] = packet[headerIndex + i];
			  }

		  }

		  if(curPacket[0] == 0x54 && curPacket[1] == 0x2c)//Don't do anything else if header and version are not correct
		  {
			  crc = CalCRC8(curPacket, 46);

		  }

		  if(crc == curPacket[46])//Calculated CRC needs to equal CRC8 received in packet (last byte of packet)
		  {
			  //Data is good
			  goodPackets++;
			  if(!SEND_MOTOR_VALS)//Dont send LiDAR data if sending motor vals
				  getLiDARMeasurements(curPacket, RadioTX, fullDataPacket);//Store distance and angle measurements
			  //RadioTX: {startAngle, startAngle2, endAngle, endAngle2, 12 bytes of scaled distance measurements}
			  //RadioTX: {startAngle, startAngle2, endAngle, endAngle2, 24 bytes of distance measurements, 4 bytes free} -- Old
			  //fullDataPacket: {startAngle, endAngle, data1 - data12}
			  obstacleDetection(fullDataPacket, yawEst);

		  }
		  else
		  {
			  //Data is not good
			  if(!SEND_MOTOR_VALS)
			  {
				  RadioTX[0] = 0xff;//Error
				  RadioTX[1] = 0xff;//Error
			  }
			  badPackets++;
		  }
		  totalPackets++;

		  //Swapping current and next packets
		  packetFlag = ~packetFlag;

		  if(packetFlag)
		  {
			  curPacket = packet1;
			  nextPacket = packet0;
		  }
		  else
		  {
			  curPacket = packet0;
			  nextPacket = packet1;
		  }
	  }

	  //Could awaken TX task here if need to synchronize

      osDelay(5);
  }
  /* USER CODE END lidarRX */
}

/* USER CODE BEGIN Header_readPressureSensor */
/**
* @brief Function implementing the taskPressureSen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readPressureSensor */
void readPressureSensor(void *argument)
{
  /* USER CODE BEGIN readPressureSensor */
	//__disable_irq();
	uint8_t numSums = 10;				//Number of sums for moving avg filter
	float altimeterReadings[10] = {0.0f};//Array to store readings for sums
	float altimeterSum = 0.0f;			//Sum of readings
	uint8_t altimeterSumsIndex = 0;		//Index of reading to be replaced
	uint8_t pressureSensorTX[2] = {0x50, 0x59};
	uint8_t pressureSensorRX[9] = {};			//(Pressure MMSB, MLSB, Checksum, LMSB, LLSB, Checksum, Temp MSB, LSB, Checksum)
	uint32_t rawPressureData = 0;
	uint32_t rawTempData = 0;
	float height = 0.0;
	float pressure = 0.0;
	//__enable_irq();
  /* Infinite loop */
  while(1)
  {
	  HAL_I2C_Master_Transmit(&hi2c1, ICP_ADDR, pressureSensorTX, 2, 1000);//Transmit device addr + 16-bit cmd
	  osDelay((24/portTICK_PERIOD_MS));								       //Low Noise conv guranteed to finish in 23.8ms
	  HAL_I2C_Master_Receive(&hi2c1, ICP_ADDR, pressureSensorRX, 9, 1000); //Transmit device addr + read and then RX 9 bytes of data

	  //CRC Checks:(Doesn't seem to need, but would likely be good to implement)

	  //For pressure, only MMSB, MLSB, and LMSB matter, LLSM can be disregarded
	  rawPressureData = pressureSensorRX[0] << 16 | pressureSensorRX[1] << 8 | pressureSensorRX[3];
	  rawTempData = pressureSensorRX[6] << 8 | pressureSensorRX[7];
	  inv_invpres_process_data(&icp_data, rawPressureData , rawTempData, &height, &pressure);

	  //Moving average filter
	  altimeterSum = altimeterSum - altimeterReadings[altimeterSumsIndex] + height;
	  altimeterReadings[altimeterSumsIndex] = height;
	  height = altimeterSum / numSums;
	  altimeterSumsIndex = (altimeterSumsIndex + 1) % numSums;

	  osMessageQueuePut(pressureSensorQueueHandle, &height, 0U, 0U);

	  //printf("Altimeter Height: %0.3f\r\n", height);
	  //Send height data somewhere
	  osDelay(76);	//round out 100ms of delay
  }
  /* USER CODE END readPressureSensor */
}

/* USER CODE BEGIN Header_measureHeight */
/**
* @brief Function implementing the taskRangeFinder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_measureHeight */
void measureHeight(void *argument)
{
  /* USER CODE BEGIN measureHeight */
	__disable_irq();
	uint8_t numSums = 5;			//Number of sums for moving avg filter
	float rfReadings[5] = {0.0f};	//Array to store readings for sums
	float rfSum = 0.0f;				//Sum of readings
	uint8_t rfSumsIndex = 0;		//Index of reading to be replaced
	uint8_t rangeFinderBuffer[24] = {0};
	float rangeFinderHeight = 0.0f;
	float altimeterHeight = 0.0f;
	uint16_t rf_measurement = 0;
	uint8_t initMeasurements = 0;
	float altimeterOffset = 0.0f;
	float combinedHeight = 0.0f;
	uint32_t crc = 0;
	uint32_t expectedCRC = 1;
	uint8_t testMsg[12] = {0xFA, 0xE1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x63, 0x08, 0x6D};

	RF_SetFiltering(rangeFinderBuffer);						//Turn on filter
	RF_SetSamplePeriod_100ms(rangeFinderBuffer);			//Set sample rate to 200ms
	RF_SetContMode(rangeFinderBuffer);     					//Set to continuous capture mode
	RF_GetMeasurement(rangeFinderPacket);					//Start measuring
	//HAL_UART_Receive_DMA(&huart5, rangeFinderPacket, 24);	//Start UART RX DMA for range finder data
	__enable_irq();
  /* Infinite loop */
    while(1)
    {
	  if(osMessageQueueGetCount(rangeFinderQueueHandle))
	  {
		  osMessageQueueGet(rangeFinderQueueHandle, rangeFinderBuffer, 0U, 0U);
		  crc = crcFast(rangeFinderBuffer, 20);
		  expectedCRC = (rangeFinderBuffer[23] << 24 | rangeFinderBuffer[22] << 16 | rangeFinderBuffer[21] << 8 | rangeFinderBuffer[20]);
		  /*if(crc == expectedCRC)
			  printf("Data Valid\r\n");
		  else
			  printf("Data Invalid\r\n");
		  printf("Result: 0x%x, Expected: 0x%x\r\n", crc, expectedCRC);*///Try setting task prio higher so it doesnt get preemted and see if that fixes it
		  rf_measurement = (rangeFinderBuffer[5] << 8) | rangeFinderBuffer[4];
		  if(rf_measurement < 12000)
		  {
			  rangeFinderHeight = (float)(rf_measurement) / 1000.0f;
			  //Moving average filter
			  rfSum = rfSum - rfReadings[rfSumsIndex] + rangeFinderHeight;
			  rfReadings[rfSumsIndex] = rangeFinderHeight;
			  rangeFinderHeight = rfSum / numSums;
			  rfSumsIndex = (rfSumsIndex + 1) % numSums;
		  }
		  //printf("RF Height: %0.3f\r\n", rangeFinderHeight);
		  //Use a queue to send wherever needed.
	  }

	  if(osMessageQueueGetCount(pressureSensorQueueHandle))
	  {
		  osMessageQueueGet(pressureSensorQueueHandle, &altimeterHeight, 0U, 0U);
		  if(initMeasurements < 10)
		  {
			  initMeasurements++;
			  //altimeterOffset = (altimeterOffset * (initMeasurements - 1) + altimeterHeight) / initMeasurements;
			  altimeterOffset += altimeterHeight;
			  if(initMeasurements == 10)
			  {
				  altimeterOffset = altimeterOffset/initMeasurements;
				  altimeterOffset += altimeterHeight - rangeFinderHeight;
			  }
				  //altimeterOffset = fabsf(altimeterOffset - rangeFinderHeight);
		  }
		  else
		  {
			  //Seems to sometimes be off by about 0.3 - 0.4 (above rf height)
			  altimeterHeight = altimeterHeight - altimeterOffset;
			  //printf("Altimeter Height: %0.3f\r\n", altimeterHeight);
		  }
	  }

	  if(rangeFinderHeight >= 12.0f || rangeFinderHeight < 0.0f)
		  rangeFinderHeight = 12.0f;	//Ignore rangefinder data if it is negative or outside operation range
	  //Weighted average; rf is used more closer to the ground, and pressure sensor is used more further away
	  combinedHeight = ((altimeterHeight * (rangeFinderHeight / 12.0f)) + (rangeFinderHeight * (1.0f - rangeFinderHeight/12.0f)));
	  //printf("RF: %0.3f, Altimeter: %0.3f, Combined: %0.3f, \r\n\r\n", rangeFinderHeight, altimeterHeight, combinedHeight);

      osDelay(100);
    }
  /* USER CODE END measureHeight */
}

/* USER CODE BEGIN Header_readIMU */
/**
* @brief Function implementing the taskReadIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readIMU */
void readIMU(void *argument)
{
  /* USER CODE BEGIN readIMU */
	//__disable_irq();
	uint8_t acc_data_rdy = 0;
	uint8_t gyro_data_rdy = 0;
	float velocities[3] = {0.0f};
	float velEst = 0.0f;
	float roll, pitch, dt;
	TickType_t curTicks_g, curTicks_a, prevTicks_g, prevTicks_a;
	DroneRotations rotationVals = {0,0,0};
	dev.delay_ms = &osDelay;//don't do this if using IMU interrupts, otherwise, all other IMU
							//stuff should be from tasks

	//struct eulerAngles droneAngles = {0,0,0};//For complementary filter

	KalmanFilterRollPitch extKalmanFilter;
	float Q[2] = {3.0f * EKF_N_GYR, 2.0f * EKF_N_GYR};
	float R[3] = {EKF_N_ACC, EKF_N_ACC, EKF_N_ACC};
	EKF_Init(&extKalmanFilter, EKF_P_INIT, Q, R);//HERE
	FIRFilter filter[6];
	for(int i = 0; i < 6; ++i)
		FIRFilter_Init(&filter[i]);	//Acc x, y, z, Gyro x, y, z

	HAL_TIM_Base_Start_IT(&htim6);//Start timer for yaw angle est
	//__enable_irq();
  /* Infinite loop */
	while(1)
	{

		  if(!(acc_data_rdy & 0x80))
			  bmi08a_get_regs(BMI08X_ACCEL_STATUS_REG, &acc_data_rdy, 1, &dev);

		  if(!(gyro_data_rdy & 0x80))
			  bmi08g_get_regs(BMI08X_GYRO_INT_STAT_1_REG, &gyro_data_rdy, 1, &dev);

		  if((gyro_data_rdy & 0x80))
		  {
			  bmi08g_get_data(&user_gyro_bmi088, &dev);

			  user_gyro_bmi088.x = FIRFilter_Update(&filter[3], user_gyro_bmi088.x);
			  user_gyro_bmi088.y = FIRFilter_Update(&filter[4], user_gyro_bmi088.y);
			  user_gyro_bmi088.z = FIRFilter_Update(&filter[5], user_gyro_bmi088.z);

			  /*yawEst = (yawEst + ((htim6.Instance->CNT)/10000.0f * user_gyro_bmi088.z));// CCW = positive val, CW = negative val (due to IMU orientation)
			  htim6.Instance->CNT = 0;
			  if(yawEst < -360.0f)
				  yawEst += 360.0f;
			  else if(yawEst > 360.0f)
				  yawEst -= 360.0f;*/

			 // printf("Gyro - X: %f, Y: %f, Z: %f\r\n", user_gyro_bmi088.x, user_gyro_bmi088.y, user_gyro_bmi088.z);
			  curTicks_g = xTaskGetTickCount();
			  dt = (float)((float)(curTicks_g - prevTicks_g) / 1000.0f);
			  prevTicks_g = curTicks_g;
			  //printf("Gyro: %0.3f\r\n", dt);
			  //Kalman filter predict
			  EKF_Predict(&extKalmanFilter, &user_gyro_bmi088, dt);;
		  }

		  if((acc_data_rdy & 0x80) && (gyro_data_rdy & 0x80))//want gyro to go first to predict and then update
		  {
			  bmi08a_get_data(&user_accel_bmi088, &dev);
			  user_accel_bmi088.x = FIRFilter_Update(&filter[0], user_accel_bmi088.x);
			  user_accel_bmi088.y = FIRFilter_Update(&filter[1], user_accel_bmi088.y);
			  user_accel_bmi088.z = FIRFilter_Update(&filter[2], user_accel_bmi088.z);

			  //printf("AX: %0.3f, AY: %0.3f, AZ: %0.3f, GX: %0.3f, GY: %0.3f, GZ: %0.3f\r\n", user_accel_bmi088.x, user_accel_bmi088.y, user_accel_bmi088.z, user_gyro_bmi088.x, user_gyro_bmi088.y, user_gyro_bmi088.z);
			  curTicks_a = xTaskGetTickCount();
			  dt = (float)((float)(curTicks_a - prevTicks_a) / 1000.0f);
			  prevTicks_a = curTicks_a;
			  //printf("Acc: %0.3f\r\n", dt);
			  //Kalman filter update
			  EKF_Update(&extKalmanFilter, &user_accel_bmi088, dt);//Angles stored in radians //HERE
			  //KalmanRollPitch_Update(&extKalmanFilter, acc, 0.005);
//			  printf("Roll: %0.3f, Pitch: %0.3f, Yaw Rate: %0.3f\r\n", extKalmanFilter.roll * RAD_TO_DEG, extKalmanFilter.pitch * RAD_TO_DEG, user_gyro_bmi088.z);
			  gyro_data_rdy = 0;
			  acc_data_rdy = 0;

			  //velEst = VelocityEstimate(velocities, &user_accel_bmi088, &user_gyro_bmi088, 0.005f);
			  //printf("Velocity: %0.3f\r\nVX: %0.3f, VY: %0.3f, VZ: %0.3f\r\n", velEst, velocities[0], velocities[1], velocities[2]);
			  //yawEst += 0.04f * (sin(extKalmanFilter.roll) * user_gyro_bmi088.y + cos(extKalmanFilter.roll) * user_gyro_bmi088.z) / cos(extKalmanFilter.pitch);
			  //yawEst = (yawEst + 0.04f*(sinf(extKalmanFilter.roll) * user_gyro_bmi088.y + cosf(extKalmanFilter.roll) * user_gyro_bmi088.z) / cosf(extKalmanFilter.pitch));
			  //printf("YawEst: %0.3f\r\n", yawEst * RAD_TO_DEG);
		  }
		  rotationVals.pitch = extKalmanFilter.pitch * RAD_TO_DEG - 1.40670f; //Subtract IMU error (not level)
		  rotationVals.roll = extKalmanFilter.roll * RAD_TO_DEG - 1.44123f;
		  rotationVals.rollRate = user_gyro_bmi088.x;
		  rotationVals.pitchRate = -user_gyro_bmi088.y;//Pitch needs to be negative. Not sure why it is different
		  rotationVals.yawRate = user_gyro_bmi088.z;
//		  rotationVals.pitch = extKalmanFilter.pitch * RAD_TO_DEG;
//		  rotationVals.roll = extKalmanFilter.roll * RAD_TO_DEG;
		  //printf("Roll: %0.3f, Pitch: %0.3f, Yaw Rate: %0.3f\r\n", rotationVals.roll, rotationVals.pitch, user_gyro_bmi088.z);


		  //Testing w/o queue
//		  globalRotationVals.yawRate = user_gyro_bmi088.z;
//		  globalRotationVals.pitch = extKalmanFilter.pitch * RAD_TO_DEG;
//		  globalRotationVals.roll = extKalmanFilter.roll * RAD_TO_DEG;

		  if(SEND_MOTOR_VALS)
		  {
			  RadioTX[8] = (int8_t)(rotationVals.yaw * 100.0f);
			  RadioTX[9] = (int8_t)(rotationVals.pitch * 100.0f);
			  RadioTX[10] = (int8_t)(rotationVals.roll * 100.0f);
		  }

		  osMessageQueuePut(rotationsQueueHandle, (DroneRotations*)&rotationVals, 0U, 0U);

		  osDelay(3);
	}
  /* USER CODE END readIMU */
}

/* USER CODE BEGIN Header_taskUpdatePIDs */
/**
* @brief Function implementing the UpdatePIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskUpdatePIDs */
void taskUpdatePIDs(void *argument)
{
  /* USER CODE BEGIN taskUpdatePIDs */
	//__disable_irq();
	DroneRotations rotationVals = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	motor_values motor_vals = {0,0,0,0};
	DShotVals finalThrottleVals = {0,0,0,0};
	DShotVals CA_Update = {0,0,0,0};
	uint16_t throttleCorrection = 0;
	float dt = 0.001f;
	TickType_t prevTicks = 0;
	TickType_t curTicks = 0;

	uint16_t testThrottle = 400;

	//Controllers for two-layer controller: (Angle controller for roll and pitch) -> (Angle rate controller for roll and pitch) -> Update Motors
	PIDController RollController = {0.5f, 0.5f, 0.5f, MAX_ROLL_ANGLE, -MAX_ROLL_ANGLE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	PIDController PitchController = {0.5f, 0.5f, 0.5f, MAX_PITCH_ANGLE, -MAX_PITCH_ANGLE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	PIDController YawRateController = {2.5f, 0.0f, 1.0f, MAX_YAW_RATE, -MAX_YAW_RATE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	PIDController RollRateController = {1.0f, 0.0f, 0.0f, MAX_ROLL_RATE, -MAX_ROLL_RATE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	PIDController PitchRateController = {1.0f, 0.0f, 0.0f, MAX_PITCH_RATE, -MAX_PITCH_RATE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	//Controllers for testing CA:
//	PIDController RollController = {0.5f, 0.0f, 0.0f, MAX_ROLL_ANGLE, -MAX_ROLL_ANGLE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//	PIDController PitchController = {0.5f, 0.0f, 0.0f, MAX_PITCH_ANGLE, -MAX_PITCH_ANGLE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//	PIDController YawRateController = {2.5f, 0.0f, 0.0f, MAX_YAW_RATE, -MAX_YAW_RATE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//	PIDController RollRateController = {0.5f, 0.0f, 0.0f, MAX_ROLL_RATE, -MAX_ROLL_RATE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//	PIDController PitchRateController = {0.5f, 0.0f, 0.0f, MAX_PITCH_RATE, -MAX_PITCH_RATE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

	//Lift(Throttle) = 0.0014x^2 + 0.2126x - 5.9, R^2 = 0.9972 (All four motors)
	//Lift(Throttle) = 0.0004x^2 + 0.0513x - 1.475, R^2 = 0.9972 (One motor)

	float updateRoll = 0.0;
	float updatePitch = 0.0;
	float updateYaw = 0.0;
	uint16_t dshotVals[4] = {0,0,0,0};
	//__enable_irq();
  /* Infinite loop */
  while(1)
  {
	  if(osMessageQueueGetCount(motorControlQueueHandle))
	  {
		  osMessageQueueGet(motorControlQueueHandle, &motor_vals, NULL, 10);
	  }

	  if(osMessageQueueGetCount(rotationsQueueHandle))
	  {
		  osMessageQueueGet(rotationsQueueHandle, &rotationVals, NULL, 10);
	  }

	  if(osMessageQueueGetCount(DShotQueueHandle))
	  {
		  osMessageQueueGet(DShotQueueHandle, &CA_Update, NULL, 10);
	  }

	  finalThrottleVals.motor1 = (int16_t)motor_vals.throttle;
	  finalThrottleVals.motor2 = (int16_t)motor_vals.throttle;
	  finalThrottleVals.motor3 = (int16_t)motor_vals.throttle;
	  finalThrottleVals.motor4 = (int16_t)motor_vals.throttle;

	  //printf("Roll: %0.3f, Pitch: %0.3f, Yaw Rate: %0.3f\r\n", rotationVals.roll, rotationVals.pitch, rotationVals.yawRate);

	  if(motor_vals.throttle)//Only calculate other values if the throttle is not in deadzone
	  {
		  //Get some angle value from CA task that says which direction (or sector) to ignore JS inputs towards
		  //Does not solve all issues though. Need to somehow adjust setpoints for
		  //PIDs so they do not cancel out CA, but at the same time; the  PIDs need to stabilize the quad.

		  //Scale JS Values
		  //If changing to a base throttle model, ever increment of throttle after the deadzone should add to the base.
		  //Needs to scale so that the range of JS vals from above deadzone - max JS val maps to base + 0 to max throttle
//		  updateRoll = ((float)motor_vals.roll / MAX_JOYSTICK_VAL) * MAX_ROLL_ANGLE;
//		  updatePitch = ((float)motor_vals.pitch / MAX_JOYSTICK_VAL) * MAX_PITCH_ANGLE;
//		  updateYaw = ((float)motor_vals.yaw / MAX_JOYSTICK_VAL) * MAX_YAW_RATE;

		  //Only Rate Controllers:
		  updateRoll = ((float)motor_vals.roll / MAX_JOYSTICK_VAL) * MAX_ROLL_RATE;
		  updatePitch = ((float)motor_vals.pitch / MAX_JOYSTICK_VAL) * MAX_PITCH_RATE;
		  updateYaw = ((float)motor_vals.yaw / MAX_JOYSTICK_VAL) * MAX_YAW_RATE;

		  //printf("%0.3f, %0.3f, %d, %0.3f, %0.3f\r\n", rotationVals.pitch, updatePitch, 0, rotationVals.roll, updateRoll);//Graph setpoints and current angle

		  printf("SETPOINTS: Roll: %0.3f, Pitch %0.3f, Yaw %0.3f\r\n", updateRoll, updatePitch, updateYaw);
		  printf("CURRENT ANGLE: Roll: %0.3f, Pitch: %0.3f, Yaw Rate: %0.3f\r\n", rotationVals.roll, rotationVals.pitch, rotationVals.yawRate);
		  printf("CURRENT RATE: Roll Rate: %0.3f, Pitch Rate: %0.3f, Yaw Rate: %0.3f\r\n", rotationVals.rollRate, rotationVals.pitchRate, rotationVals.yawRate);
		  //Update PID Controllers

		  //Two-layer Controller:
		  //Angle Controllers:
		  //updateRoll = PID_Update(&RollController, rotationVals.roll, updateRoll, 0.006f)*1.0f;	   //Out used as input into rate controller
		  //updatePitch = PID_Update(&PitchController, rotationVals.pitch, updatePitch, 0.006f)*1.0f; //Out used as input into rate controller

		  //Convert angle updates to angle rate setpoints:
		  //updateRoll = updateRoll*ANGLE_TO_RATE_CONV;//Linear mapping, may want to find a different method
		  //updatePitch = updatePitch*ANGLE_TO_RATE_CONV;
		  curTicks = xTaskGetTickCount();
		  dt = (float)((float)(curTicks - prevTicks) / 1000.0f);
		  prevTicks = curTicks;
		  //printf("%0.3f\r\n", dt);
		  //Rate Controllers:
		  updateRoll = PID_Update(&RollRateController, rotationVals.rollRate, updateRoll, dt);	   //Out used as input into rate controller
		  updatePitch = PID_Update(&PitchRateController, rotationVals.pitchRate, updatePitch, dt); //Out used as input into rate controller
		  updateYaw = PID_Update(&YawRateController, rotationVals.yawRate, updateYaw, dt);				   //Yaw update is taken directly
		  printf("PID UPDATES: Roll: %0.3f, Pitch %0.3f, Yaw %0.3f\r\n", updateRoll, updatePitch, updateYaw);
		  //printf("Roll: %0.3f, Pitch %0.3f, Yaw %0.3f\r\n", updateRoll - rotationVals.roll, updatePitch - rotationVals.pitch, updateYaw - rotationVals.yaw);
//		  updateRoll = 0.0f;
//		  updatePitch = 0.0f;
//		  updateYaw = 0.0f;

		  //Update Throttle Values for Each Motor
		  //These are ints, not uint b/c do not want it to go negative and end up sending max throttle
		  finalThrottleVals.motor1 = (int16_t)roundf((float)finalThrottleVals.motor1 - updatePitch - updateRoll + updateYaw);
		  finalThrottleVals.motor2 = (int16_t)roundf((float)finalThrottleVals.motor2 + updatePitch - updateRoll - updateYaw);
		  finalThrottleVals.motor3 = (int16_t)roundf((float)finalThrottleVals.motor3 - updatePitch + updateRoll - updateYaw);
		  finalThrottleVals.motor4 = (int16_t)roundf((float)finalThrottleVals.motor4 + updatePitch + updateRoll + updateYaw);

		  finalThrottleVals.motor1 = (int16_t)roundf((float)finalThrottleVals.motor1 + CA_Update.motor1);
		  finalThrottleVals.motor2 = (int16_t)roundf((float)finalThrottleVals.motor2 + CA_Update.motor2);
		  finalThrottleVals.motor3 = (int16_t)roundf((float)finalThrottleVals.motor3 + CA_Update.motor3);
		  finalThrottleVals.motor4 = (int16_t)roundf((float)finalThrottleVals.motor4 + CA_Update.motor4);
		  printf("THROTTLES: M1: %d M3: %d M2: %d M4: %d\r\n", finalThrottleVals.motor1, finalThrottleVals.motor3, finalThrottleVals.motor2, finalThrottleVals.motor4);
		  //printf("CA Updates: M1: %d, M3: %d, M2: %d, M4: %d\r\n", CA_Update.motor1, CA_Update.motor3, CA_Update.motor2, CA_Update.motor4);

		  if(finalThrottleVals.motor1 > 2047 || finalThrottleVals.motor2 > 2047 || finalThrottleVals.motor3 > 2047 || finalThrottleVals.motor4 > 2047)
			  printf("Throttle Overshoot.\r\n");

		  //Clamp throttle
//		  if(finalThrottleVals.motor1 > 2047)
//		  {
//			  throttleCorrection = finalThrottleVals.motor1 - 2047;
//			  finalThrottleVals.motor1 = finalThrottleVals.motor1 - throttleCorrection;
//			  finalThrottleVals.motor2 = finalThrottleVals.motor2 - throttleCorrection;
//			  finalThrottleVals.motor3 = finalThrottleVals.motor3 - throttleCorrection;
//			  finalThrottleVals.motor4 = finalThrottleVals.motor4 - throttleCorrection;
//		  }
//		  else if(finalThrottleVals.motor1 < 0)
//			  finalThrottleVals.motor1 = 0;
//
//		  if(finalThrottleVals.motor2 > 2047)
//		  {
//			  throttleCorrection = finalThrottleVals.motor2 - 2047;
//			  finalThrottleVals.motor1 = finalThrottleVals.motor1 - throttleCorrection;//Subtract throttle overshoot from all motors
//			  finalThrottleVals.motor2 = finalThrottleVals.motor2 - throttleCorrection;
//			  finalThrottleVals.motor3 = finalThrottleVals.motor3 - throttleCorrection;
//			  finalThrottleVals.motor4 = finalThrottleVals.motor4 - throttleCorrection;
//		  }
//		  else if(finalThrottleVals.motor2 < 0)
//			  finalThrottleVals.motor2 = 0;
//
//		  if(finalThrottleVals.motor3 > 2047)
//		  {
//			  throttleCorrection = finalThrottleVals.motor3 - 2047;
//			  finalThrottleVals.motor1 = finalThrottleVals.motor1 - throttleCorrection;
//			  finalThrottleVals.motor2 = finalThrottleVals.motor2 - throttleCorrection;
//			  finalThrottleVals.motor3 = finalThrottleVals.motor3 - throttleCorrection;
//			  finalThrottleVals.motor4 = finalThrottleVals.motor4 - throttleCorrection;
//		  }
//		  else if(finalThrottleVals.motor3 < 0)
//			  finalThrottleVals.motor3 = 0;
//
//		  if(finalThrottleVals.motor4 > 2047)
//		  {
//			  throttleCorrection = finalThrottleVals.motor4 - 2047;
//			  finalThrottleVals.motor1 = finalThrottleVals.motor1 - throttleCorrection;
//			  finalThrottleVals.motor2 = finalThrottleVals.motor2 - throttleCorrection;
//			  finalThrottleVals.motor3 = finalThrottleVals.motor3 - throttleCorrection;
//			  finalThrottleVals.motor4 = finalThrottleVals.motor4 - throttleCorrection;
//		  }
//		  else if(finalThrottleVals.motor4 < 0)
//			  finalThrottleVals.motor4 = 0;

		  	  	  testThrottle = 700;

				  if(finalThrottleVals.motor1 > testThrottle || finalThrottleVals.motor2 > testThrottle || finalThrottleVals.motor3 > testThrottle || finalThrottleVals.motor4 > testThrottle)
					  printf("Check\r\n");

		  		  if(finalThrottleVals.motor1 > testThrottle)
		  		  {
		  			  throttleCorrection = finalThrottleVals.motor1 - testThrottle;
		  			  finalThrottleVals.motor1 = finalThrottleVals.motor1 - throttleCorrection;
		  			  finalThrottleVals.motor2 = finalThrottleVals.motor2 - throttleCorrection;
		  			  finalThrottleVals.motor3 = finalThrottleVals.motor3 - throttleCorrection;
		  			  finalThrottleVals.motor4 = finalThrottleVals.motor4 - throttleCorrection;
		  		  }
		  		  else if(finalThrottleVals.motor1 < 0)
		  			  finalThrottleVals.motor1 = 0;

		  		  if(finalThrottleVals.motor2 > testThrottle)
		  		  {
		  			  throttleCorrection = finalThrottleVals.motor2 - testThrottle;
		  			  finalThrottleVals.motor1 = finalThrottleVals.motor1 - throttleCorrection;//Subtract throttle overshoot from all motors
		  			  finalThrottleVals.motor2 = finalThrottleVals.motor2 - throttleCorrection;
		  			  finalThrottleVals.motor3 = finalThrottleVals.motor3 - throttleCorrection;
		  			  finalThrottleVals.motor4 = finalThrottleVals.motor4 - throttleCorrection;
		  		  }
		  		  else if(finalThrottleVals.motor2 < 0)
		  			  finalThrottleVals.motor2 = 0;

		  		  if(finalThrottleVals.motor3 > testThrottle)
		  		  {
		  			  throttleCorrection = finalThrottleVals.motor3 - testThrottle;
		  			  finalThrottleVals.motor1 = finalThrottleVals.motor1 - throttleCorrection;
		  			  finalThrottleVals.motor2 = finalThrottleVals.motor2 - throttleCorrection;
		  			  finalThrottleVals.motor3 = finalThrottleVals.motor3 - throttleCorrection;
		  			  finalThrottleVals.motor4 = finalThrottleVals.motor4 - throttleCorrection;
		  		  }
		  		  else if(finalThrottleVals.motor3 < 0)
		  			  finalThrottleVals.motor3 = 0;

		  		  if(finalThrottleVals.motor4 > testThrottle)
		  		  {
		  			  throttleCorrection = finalThrottleVals.motor4 - testThrottle;
		  			  finalThrottleVals.motor1 = finalThrottleVals.motor1 - throttleCorrection;
		  			  finalThrottleVals.motor2 = finalThrottleVals.motor2 - throttleCorrection;
		  			  finalThrottleVals.motor3 = finalThrottleVals.motor3 - throttleCorrection;
		  			  finalThrottleVals.motor4 = finalThrottleVals.motor4 - throttleCorrection;
		  		  }
		  		  else if(finalThrottleVals.motor4 < 0)
		  			  finalThrottleVals.motor4 = 0;

//		  finalThrottleVals.motor1 = 800;
//		  finalThrottleVals.motor2 = 800;
//		  finalThrottleVals.motor3 = 800;
//		  finalThrottleVals.motor4 = 800;

		  printf("PID ADJS: M1: %0.3f M3: %0.3f M2: %0.3f M4: %0.3f\r\n\r\n", (-updatePitch - updateRoll + updateYaw), (-updatePitch + updateRoll - updateYaw), (updatePitch - updateRoll - updateYaw),(updatePitch + updateRoll + updateYaw));
	  }
	  if(SEND_MOTOR_VALS)
	  {
		  RadioTX[0] = (finalThrottleVals.motor1 >> 8);
		  RadioTX[1] = (finalThrottleVals.motor1 & 0x00FF);
		  RadioTX[2] = (finalThrottleVals.motor2 >> 8);
		  RadioTX[3] = (finalThrottleVals.motor2 & 0x00FF);
		  RadioTX[4] = (finalThrottleVals.motor3 >> 8);
		  RadioTX[5] = (finalThrottleVals.motor3 & 0x00FF);
		  RadioTX[6] = (finalThrottleVals.motor4 >> 8);
		  RadioTX[7] = (finalThrottleVals.motor4 & 0x00FF);
	  }
	  dshotVals[0] = finalThrottleVals.motor1;
	  dshotVals[1] = finalThrottleVals.motor2;
	  dshotVals[2] = finalThrottleVals.motor3;
	  dshotVals[3] = finalThrottleVals.motor4;

	  dshot_write(dshotVals);
	  //Send throttle values
	  //osMessageQueuePut(DShotQueueHandle, (DShotVals*)&finalThrottleVals, 0U, 0U);//ADD BACK IF NOT UPDATING MOTORS HERE
	  osDelay(2);
	  //printf("Tick Rate in HZ: %d\r\n", configTICK_RATE_HZ);
	  //vTaskDelay(100 / DELAY_US);//Not sure this will work. Seems that the lowest tick rate is 1ms. Does not error though.
  }
  /* USER CODE END taskUpdatePIDs */
}

/* USER CODE BEGIN Header_collisionAvoidance */
/**
* @brief Function implementing the CA_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_collisionAvoidance */
void collisionAvoidance(void *argument)
{
  /* USER CODE BEGIN collisionAvoidance */
  //__disable_irq();
  PIDController sectorControllers[NUM_SECTORS];
  DShotVals CA_MotorUpdates = {0, 0, 0, 0};
  uint16_t sectorAdjustments[NUM_SECTORS] = {0};
  float sampleTime = 0.05f;
  float updateVal = 0.0f;
  const float onePart = 1.0f / NUM_SECTORS;
  const float threeParts = 3.0f / NUM_SECTORS;
  initCA(sectorControllers, 1.0f, 0.0f, 0.0f);
  //__enable_irq();
  /* Infinite loop */
  while(1)//Make sure task has 512 bytes. 1024 seems to break it
  {
	//Call hazard assessment
	  //hazardAssessment(yawEst, 0.0f, 0.0f);
	//Update PID controllers
	obstacleAvoidance(sectorControllers, sectorAdjustments, sampleTime);
	//Update motor values
	//Since each sector has the angle of the closest obstacle, should base motor adjustments based on which motors the angle is closest to
	updateVal = onePart*sectorAdjustments[0] - onePart*sectorAdjustments[1] - threeParts*sectorAdjustments[2] - threeParts*sectorAdjustments[3]
			   - onePart*sectorAdjustments[4] + onePart*sectorAdjustments[5] + threeParts*sectorAdjustments[6] + threeParts*sectorAdjustments[7];
	CA_MotorUpdates.motor1 = (int16_t)round(updateVal);

	updateVal = -threeParts*sectorAdjustments[0] - threeParts*sectorAdjustments[1] - onePart*sectorAdjustments[2] + onePart*sectorAdjustments[3]
			   + threeParts*sectorAdjustments[4] + threeParts*sectorAdjustments[5] + onePart*sectorAdjustments[6] - onePart*sectorAdjustments[7];
	CA_MotorUpdates.motor2 = (int16_t)round(updateVal);

	updateVal = threeParts*sectorAdjustments[0] + threeParts*sectorAdjustments[1] + onePart*sectorAdjustments[2] - onePart*sectorAdjustments[3]
			   - threeParts*sectorAdjustments[4] - threeParts*sectorAdjustments[5] - onePart*sectorAdjustments[6] + onePart*sectorAdjustments[7];
	CA_MotorUpdates.motor3 = (int16_t)round(updateVal);


	updateVal = -onePart*sectorAdjustments[0] + onePart*sectorAdjustments[1] + threeParts*sectorAdjustments[2] + threeParts*sectorAdjustments[3]
			   + onePart*sectorAdjustments[4] - onePart*sectorAdjustments[5] - threeParts*sectorAdjustments[6] - threeParts*sectorAdjustments[7];
	CA_MotorUpdates.motor4 = (int16_t)round(updateVal);
	//Send to PID Update
	osMessageQueuePut(DShotQueueHandle, (DShotVals*)&CA_MotorUpdates, 0U, 0U);
	//printf("CA Updates: M1: %d, M2: %d, M3: %d, M4: %d\r\n", CA_MotorUpdates.motor1, CA_MotorUpdates.motor2, CA_MotorUpdates.motor3, CA_MotorUpdates.motor4);
    osDelay(10);
  }
  /* USER CODE END collisionAvoidance */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

