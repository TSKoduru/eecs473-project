/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for distanceSensor */
osThreadId_t distanceSensorHandle;
const osThreadAttr_t distanceSensor_attributes = {
  .name = "distanceSensor",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for receiveBluetooth */
osThreadId_t receiveBluetoothHandle;
const osThreadAttr_t receiveBluetooth_attributes = {
  .name = "receiveBluetooth",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for readIMU */
osThreadId_t readIMUHandle;
const osThreadAttr_t readIMU_attributes = {
  .name = "readIMU",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for dropPayload */
osThreadId_t dropPayloadHandle;
const osThreadAttr_t dropPayload_attributes = {
  .name = "dropPayload",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for controlMotors */
osThreadId_t controlMotorsHandle;
const osThreadAttr_t controlMotors_attributes = {
  .name = "controlMotors",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for parseBluetooth */
osThreadId_t parseBluetoothHandle;
const osThreadAttr_t parseBluetooth_attributes = {
  .name = "parseBluetooth",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for distance */
osSemaphoreId_t distanceHandle;
const osSemaphoreAttr_t distance_attributes = {
  .name = "distance"
};
/* Definitions for IMU */
osSemaphoreId_t IMUHandle;
const osSemaphoreAttr_t IMU_attributes = {
  .name = "IMU"
};
/* Definitions for bluetooth */
osSemaphoreId_t bluetoothHandle;
const osSemaphoreAttr_t bluetooth_attributes = {
  .name = "bluetooth"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

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
  /* creation of distance */
  distanceHandle = osSemaphoreNew(1, 1, &distance_attributes);

  /* creation of IMU */
  IMUHandle = osSemaphoreNew(1, 1, &IMU_attributes);

  /* creation of bluetooth */
  bluetoothHandle = osSemaphoreNew(1, 1, &bluetooth_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of distanceSensor */
  distanceSensorHandle = osThreadNew(distanceSensor, NULL, &distanceSensor_attributes);

  /* creation of receiveBluetooth */
  receiveBluetoothHandle = osThreadNew(receiveBluetooth, NULL, &receiveBluetooth_attributes);

  /* creation of readIMU */
  readIMUHandle = osThreadNew(readIMU, NULL, &readIMU_attributes);

  /* creation of dropPayload */
  dropPayloadHandle = osThreadNew(dropPayload, NULL, &dropPayload_attributes);

  /* creation of controlMotors */
  controlMotorsHandle = osThreadNew(controlMotors, NULL, &controlMotors_attributes);

  /* creation of parseBluetooth */
  parseBluetoothHandle = osThreadNew(parseBluetooth, NULL, &parseBluetooth_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_distanceSensor */
/**
* @brief Function implementing the distanceSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_distanceSensor */
void distanceSensor(void *argument)
{
  /* USER CODE BEGIN distanceSensor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END distanceSensor */
}

/* USER CODE BEGIN Header_receiveBluetooth */
/**
* @brief Function implementing the receiveBluetooth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_receiveBluetooth */
void receiveBluetooth(void *argument)
{
  /* USER CODE BEGIN receiveBluetooth */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END receiveBluetooth */
}

/* USER CODE BEGIN Header_readIMU */
/**
* @brief Function implementing the readIMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readIMU */
void readIMU(void *argument)
{
  /* USER CODE BEGIN readIMU */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END readIMU */
}

/* USER CODE BEGIN Header_dropPayload */
/**
* @brief Function implementing the dropPayload thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dropPayload */
void dropPayload(void *argument)
{
  /* USER CODE BEGIN dropPayload */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END dropPayload */
}

/* USER CODE BEGIN Header_controlMotors */
/**
* @brief Function implementing the controlMotors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controlMotors */
void controlMotors(void *argument)
{
  /* USER CODE BEGIN controlMotors */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END controlMotors */
}

/* USER CODE BEGIN Header_parseBluetooth */
/**
* @brief Function implementing the parseBluetooth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_parseBluetooth */
void parseBluetooth(void *argument)
{
  /* USER CODE BEGIN parseBluetooth */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END parseBluetooth */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

