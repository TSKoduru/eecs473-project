/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "Accel.h"
#include "Gyro.h"
#include "IMU.h"
#include "Vectors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_FORCE 	2.0f
#define MAX_DUTY	8.0f
#define MIN_DUTY	5.0f
#define KP			2.0f
#define KI			0.0f
#define KD			0.0f
#define ACCEL_DEADZONE  0.1f  // m/s² - tune this value
#define GYRO_DEADZONE	0.05f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
const int max_CCR = 80000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
float applyDeadzone(float val, float deadzone) {
    if (fabsf(val) < deadzone) {
        return 0.0f;
    }
    // Optional: subtract deadzone to avoid step discontinuity
    return val - copysignf(deadzone, val);
}

void worldToDroid(float *Fx_d, float *Fy_d, float F_w, float theta_w, float yaw) {
	*Fx_d = F_w * cosf(yaw - theta_w);
	*Fy_d = -F_w * sinf(yaw - theta_w);
	if (*Fx_d > 0) {
		*Fx_d = MIN(*Fx_d * sqrtf(2.0f) / 2.0f, MAX_FORCE / 2.0f);
	} else {
		*Fx_d = MAX(*Fx_d * sqrtf(2.0f) / 2.0f, MAX_FORCE / -2.0f);
	}
	if (*Fy_d > 0) {
		*Fy_d = MIN(*Fy_d * sqrtf(2.0f) / 2.0f, MAX_FORCE / 2.0f);
	} else {
		*Fy_d = MAX(*Fy_d * sqrtf(2.0f) / 2.0f, MAX_FORCE / -2.0f);
	}
}

void assignMotorValues(float Fx_d, float Fy_d, float *m1, float *m2, float *m3, float *m4) {
	*m1 = 0;
	*m2 = 0;
	*m3 = 0;
	*m4 = 0;

	// X-axis control: motors 1,3 vs 2,4
	if (Fx_d > 0) {
		*m1 += fabsf(Fx_d);
		*m3 += fabsf(Fx_d);
	} else {
		*m2 += fabsf(Fx_d);
		*m4 += fabsf(Fx_d);
	}

	// Y-axis control: motors 1,2 vs 3,4
	if (Fy_d > 0) {
		*m1 += fabsf(Fy_d);
		*m2 += fabsf(Fy_d);
	} else {
		*m3 += fabsf(Fy_d);
		*m4 += fabsf(Fy_d);
	}

	// Torque balancing: ensure diagonal pairs produce equal torque
	float T = *m1 + *m4 - *m2 - *m3;
	if (T > 0) {
		// Too much CCW torque, increase m2 and m3
		*m2 += fabsf(T) / 2.0f;
		*m3 += fabsf(T) / 2.0f;
	} else if (T < 0) {
		// Too much CW torque, increase m1 and m4
		*m1 += fabsf(T) / 2.0f;
		*m4 += fabsf(T) / 2.0f;
	}

	// Convert force to duty cycle
	float scale = (MAX_DUTY - MIN_DUTY) / MAX_FORCE;
	*m1 = *m1 * scale + MIN_DUTY;
	*m2 = *m2 * scale + MIN_DUTY;
	*m3 = *m3 * scale + MIN_DUTY;
	*m4 = *m4 * scale + MIN_DUTY;

	// Clamp to valid range
	*m1 = MAX(MIN_DUTY, MIN(*m1, MAX_DUTY));
	*m2 = MAX(MIN_DUTY, MIN(*m2, MAX_DUTY));
	*m3 = MAX(MIN_DUTY, MIN(*m3, MAX_DUTY));
	*m4 = MAX(MIN_DUTY, MIN(*m4, MAX_DUTY));
}

void TIM_SetPWM(float m1, float m2, float m3, float m4) {
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, max_CCR * 0.01 * m1);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, max_CCR * 0.01 * m2);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, max_CCR * 0.01 * m3);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, max_CCR * 0.01 * m4);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t err[] = {'e', 'r', 'r', 'o', 'r', '\n', '\r'};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //  Arm the ESC (first send the low signal then the high signal)
  TIM_SetPWM(5, 5, 5, 5);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_Delay(3000);
  IMU_INIT(&hspi1);
  IMU_SETUP_FOR_LOGGING();
  IMU_ENABLE_ALL();

  HAL_Delay(3000);

  int imuready = IMU_READY();
  while(imuready != 1) {
	  HAL_UART_Transmit(&hlpuart1, err, 7, 1000);
	  HAL_Delay(5000);
	  imuready = IMU_READY();
  }

  struct Vector3 xyz_a;
  struct Vector3 xyz_v; xyz_v.x = 0; xyz_v.y = 0; xyz_v.z = 0;
  struct Vector3 xyz_p; xyz_p.x = 0; xyz_p.y = 0; xyz_p.z = 0;

  struct Vector3 rpy_v;
  struct Vector3 rpy_p; rpy_p.x = 0; rpy_p.y = 0; rpy_p.z = 0;

  struct Vector3 xyz_ground; xyz_ground.x = 0; xyz_ground.y = 0; xyz_ground.z = 0;
  for(int i = 0; i < 5; i++) {
	  xyz_a = ACCEL_READ_ACCELERATION();
	  xyz_ground.x += xyz_a.x / 5.0f;
	  xyz_ground.y += xyz_a.y / 5.0f;
	  xyz_ground.z += xyz_a.z / 5.0f;
	  HAL_Delay(5);
  }

  float cr, sr, cp, sp, cy, sy;
  float aX, aY, aZ;
  float error_x = 0;
  float error_y = 0;
  float Fx_d, Fy_d;
  float m1, m2, m3, m4;

  uint32_t then = HAL_GetTick();
  uint32_t last = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  while ( rpy_p.x < 0.05) {
	  for (int i = 0; i < 100; i++) {
		  uint32_t now = HAL_GetTick();
		  float dt = (now - last) / 1000.0f;
		  last = now;
		  xyz_a = ACCEL_READ_ACCELERATION();
		  rpy_v = GYRO_READ_RATES();

		  xyz_a = vSub(xyz_a, xyz_ground);
		  rpy_v.x -= 0.00263f;
		  rpy_v.y += 0.001145f;
		  rpy_v.z -= 0.000884f;

		  rpy_v.x = applyDeadzone(rpy_v.x, GYRO_DEADZONE);
		  rpy_v.y = applyDeadzone(rpy_v.y, GYRO_DEADZONE);
		  rpy_v.z = applyDeadzone(rpy_v.z, GYRO_DEADZONE);

		  xyz_a.x = applyDeadzone(xyz_a.x, ACCEL_DEADZONE);
		  xyz_a.y = applyDeadzone(xyz_a.y, ACCEL_DEADZONE);
		  xyz_a.z = applyDeadzone(xyz_a.z, ACCEL_DEADZONE);

		  cr = cosf(rpy_p.x); sr = sinf(rpy_p.x);
		  cp = cosf(rpy_p.y); sp = sinf(rpy_p.y);
		  cy = cosf(rpy_p.z); sy = sinf(rpy_p.z);

		  xyz_p.x += xyz_v.x * dt;
		  xyz_p.y += xyz_v.y * dt;
		  xyz_p.z += xyz_v.z * dt;

		  aX = cy * cp * xyz_a.x + (cy * sp * sr - sy * cr) * xyz_a.y + (cy * sp * cr + sy * sr) * xyz_a.z;
		  aY = sy * cp * xyz_a.x + (sy * sp * sr + cy * cr) * xyz_a.y + (sy * sp * cr - cy * sr) * xyz_a.z;
		  aZ = -sp * xyz_a.x + (cp * sr) * xyz_a.y + (cp * cr) * xyz_a.z;

//		  aX -= xyz_ground.x;
//		  aY -= xyz_ground.y;
//		  aZ -= xyz_ground.z;


		  xyz_v.x += aX * dt;
		  xyz_v.y += aY * dt;
		  xyz_v.z += aZ * dt;

		  rpy_p.x += rpy_v.x * dt;
		  rpy_p.y += rpy_v.y * dt;
		  rpy_p.z += rpy_v.z * dt;

		  float accel_roll  = atan2f(xyz_a.y, xyz_a.z);
		  float accel_pitch = atan2f(-xyz_a.x, sqrtf(xyz_a.y*xyz_a.y + xyz_a.z*xyz_a.z));

		  float alpha = 0.99f;   // usually 0.95–0.99

		  rpy_p.x = alpha * rpy_p.x + (1.0f - alpha) * accel_roll;
		  rpy_p.y = alpha * rpy_p.y + (1.0f - alpha) * accel_pitch;

		  // Accumulate error for integral term (assuming target angle is 0)
		  error_x += xyz_p.x * dt;
		  error_y += xyz_p.y * dt;

		  // PID control to calculate world-frame forces
		  float Fx_w = -1.0*(KP * xyz_p.x + KD * xyz_v.x + KI * error_x) / 100.0;
		  float Fy_w = -1.0*(KP * xyz_p.y + KD * xyz_v.y + KI * error_y) / 100.0;

		  float F_w = sqrtf(Fx_w * Fx_w + Fy_w * Fy_w);
		  float theta = atan2f(Fy_w, Fx_w);

		  worldToDroid(&Fx_d, &Fy_d, F_w, theta, rpy_p.z);
		  assignMotorValues(Fx_d, Fy_d, &m1, &m2, &m3, &m4);

		  TIM_SetPWM(m1, m2, m3, m4);

		  HAL_Delay(5);
	  }
	  char msg[100];
	  snprintf(msg, sizeof(msg), "%.3f,%.3f,%.3f,%.3f\n\r", xyz_p.x, xyz_p.y, rpy_p.x, rpy_p.y);
//	  snprintf(msg, sizeof(msg), "%d\n\r", (then-last));
	  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	  snprintf(msg, sizeof(msg), "%.3f,%.3f,%.3f,%.3f\n\r", m1, m2, m3, m4);
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	  TIM_SetPWM(m1, m2, m3, m4);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 80000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
