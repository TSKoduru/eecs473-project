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
#include "stdio.h"

// Distance sensor
#include "../../Drivers/BSP/Components/vl53l1x_uld/VL53L1X_api.h"
#include "../../Drivers/BSP/Components/vl53l1x_uld/VL53L1X_calibration.h"

// IMU
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// NOTE: isInterrupt = 1 then device is working in interrupt mode, otherwise it is working in polling mode
#define isInterrupt 0

// NOTE: isInterrupt = 1 then device is working in interrupt mode, otherwise it is working in polling mode
#define isInterrupt 0
#define MAX_FORCE 	2.0f
#define MAX_DUTY	8.0f
#define MIN_DUTY	5.0f
#define ACCEL_DEADZONE  0.1f  // m/s² - tune to reduce drift
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
// TODO: update distance sensor to use custom address
uint16_t dev = 0x52;
int status = 0;
const int max_CCR = 20000;
uint8_t xb[3];
float KP = 2.0;
float KI = 0.0;
float KD = 0.0;
int recalibrate = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void Error_Handler(void);
// TODO: add these functions!
//void RangingLoop(void); /* */
//void Multizones(void);

float applyAccelDeadzone(float accel, float deadzone) {
	if (fabsf(accel) < deadzone) {
		return 0.0f;
	}
	return accel;
}

void assignMotorValues(float Fx_d, float Fy_d, float *m1, float *m2, float *m3,
		float *m4) {
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
uint8_t err[] = { 'e', 'r', 'r', 'o', 'r', '\n', '\r' };

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	uint8_t byteData;
	uint16_t wordData;
	uint8_t ToFSensor = 0; // 0=Left, 1=Center(default), 2=Right

	uint8_t sensorState = 0;
	uint16_t Distance;
	uint16_t SignalRate;
	uint16_t AmbientRate;
	uint16_t SpadNum;
	uint8_t RangeStatus;
	uint8_t dataReady = 0;
	uint16_t timeout_counter = 0;
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
	MX_I2C2_Init();
	MX_LPUART1_UART_Init();
	MX_I2C1_Init();
	MX_UART4_Init();
	MX_SPI1_Init();
	MX_SPI3_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */
	//  Arm the ESC (first send the low signal then the high signal)
	    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, max_CCR * 0.05);
	    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, max_CCR * 0.05);
	    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, max_CCR * 0.05);
	    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, max_CCR * 0.05);

	    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

	// Setup the Servos
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, max_CCR * 0.03);
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, max_CCR * 0.03);

	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	    HAL_Delay(5000);

//	START OF IMU SETUP
	IMU_INIT(&hspi3);
	IMU_SETUP_FOR_LOGGING();
	IMU_ENABLE_ALL();

	HAL_Delay(3000);

	int imuready = IMU_READY();
	while (imuready != 1) {
		imuready = IMU_READY();
		// HAL_UART_Transmit(&hlpuart1, err, 7, 1000);
		HAL_Delay(5000);
	}

	struct Vector3 xyz_a;
	struct Vector3 xyz_v;
	xyz_v.x = 0;
	xyz_v.y = 0;
	xyz_v.z = 0;
	struct Vector3 xyz_p;
	xyz_p.x = 0;
	xyz_p.y = 0;
	xyz_p.z = 0;

	struct Vector3 rpy_v;
	struct Vector3 rpy_p;
	rpy_p.x = 0;
	rpy_p.y = 0;
	rpy_p.z = 0;

	struct Vector3 xyz_ground;
	xyz_ground.x = 0;
	xyz_ground.y = 0;
	xyz_ground.z = 0;

//	Calibrate ground reference (should 9.81 m/s^2 in Z when stationary)
	for (int i = 0; i < 5; i++) {
		xyz_a = ACCEL_READ_ACCELERATION();
		xyz_ground.x += xyz_a.x / 5;
		xyz_ground.y += xyz_a.y / 5;
		xyz_ground.z += xyz_a.z / 5;
		HAL_Delay(5);
	}

	float cr, sr, cp, sp, cy, sy;
	float aX, aY, aZ;

	float error_x = 0;
	float error_y = 0;
	float prev_accel_x = 0;
	float prev_accel_y = 0;
	float Fx_d, Fy_d;
	float m1, m2, m3, m4;

	uint32_t lastIMU = HAL_GetTick();
	uint32_t lastDist = HAL_GetTick();
	HAL_UART_Receive_IT(&hlpuart1, &xb, 3);
//  END OF IMU SETUP

//	  START OF DISTANCE SENSOR SETUP
//  ToFSensor = 1; // Select ToFSensor: 0=Left, 1=Center, 2=Right
//  status = XNUCLEO53L1A1_ResetId(ToFSensor, 0); // Reset ToF sensor
//  HAL_Delay(2);
//  status = XNUCLEO53L1A1_ResetId(ToFSensor, 1); // Reset ToF sensor
//  HAL_Delay(2);

//    NOTE: these pins have been configured in the IOC to be high by default and have pull up resistor configured to it
//    Start up the distance sensor
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	//	TODO: Setup the other sensor, but use the default device address
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);

	/* Those basic I2C read functions can be used to check your own I2C functions */
//	status = VL53L1_RdByte(dev, 0x010F, &byteData);
//	printf("VL53L1X Model_ID: %X\n", byteData);
//	status = VL53L1_RdByte(dev, 0x0110, &byteData);
//	printf("VL53L1X Module_Type: %X\n", byteData);
//	status = VL53L1_RdWord(dev, 0x010F, &wordData);
//	printf("VL53L1X: %X\n", wordData);
//
//	while (sensorState == 0) {
//		status = VL53L1X_BootState(dev, &sensorState);
//		HAL_Delay(2);
//	}
//	printf("Chip booted\n");
//	/* This function must to be called to initialize the sensor with the default setting  */
//	status = VL53L1X_SensorInit(dev);
//	/* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//	status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//	status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//	status = VL53L1X_SetInterMeasurementInMs(dev, 100); /* in ms, IM must be > = TB */
//	status = VL53L1X_SetInterruptPolarity(dev, 0); //This function programs the interrupt polarity, 1 = active high (default), 0 = active low.
//	//  status = VL53L1X_SetOffset(dev,20); /* offset compensation in mm */
//	//  status = VL53L1X_SetROI(dev, 16, 16); /* minimum ROI 4,4 */
//
//	//	status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
//	//  printf("VL53L1X_CalibrateOffset=%d\n", offset);
//	//  status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
//	//  printf("VL53L1X_CalibrateXtalk=%d\n", xtalk/512);
//	printf("VL53L1X Ultra Lite Driver Example running ...\n");
//
//	status = VL53L1X_StartRanging(dev); /* This function has to be called to enable the ranging */
//	END OF DISTANCE SENSOR SETUP

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

//		uint32_t now = HAL_GetTick();
//		if (recalibrate) {
//			xyz_ground.x = 0;
//			xyz_ground.y = 0;
//			xyz_ground.z = 0;
//			for (int i = 0; i < 5; i++) {
//				xyz_a = ACCEL_READ_ACCELERATION();
//				xyz_ground.x += xyz_a.x / 5.0f;
//				xyz_ground.y += xyz_a.y / 5.0f;
//				xyz_ground.z += xyz_a.z / 5.0f;
//				HAL_Delay(5);
//			}
//			recalibrate = 0;
//		}
//		//		START OF DISTANCE SENSOR READ
////		if ((now - lastDist) >= 1000) {
////			lastDist = now;
////			while (dataReady == 0) {
////
////				status = VL53L1X_CheckForDataReady(dev, &dataReady);
////				timeout_counter++;
////				if (timeout_counter >= 1000) {
////					status = (uint8_t) VL53L1X_ERROR_TIMEOUT;
////					//			  printf("No data ready for long time, please check your system\n");
////					char buffer[3] = { 'e', 'r', '\n' };
////					HAL_UART_Transmit(&hlpuart1, &buffer, 3, 0xFFFF);
////					timeout_counter = 0;
////				}
////				status = VL53L1_WaitMs(dev, 1);
////			}
////			dataReady = 0;
////			timeout_counter = 0;
////
////			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
////			status = VL53L1X_GetDistance(dev, &Distance);
////			status = VL53L1X_GetSignalRate(dev, &SignalRate);
////			status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
////			status = VL53L1X_GetSpadNb(dev, &SpadNum);
////			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
////			uint8_t dist[3] = { 'A', (Distance >> 8), Distance & 0xFF };
////			// char dist[50];
////			// sprintf(dist, "A, %u \n",Distance);
////			//		printf("Range status: %u, Distance: %u mm\n", RangeStatus, Distance);
////			// HAL_UART_Transmit(&hlpuart1, dist, strlen(dist), 0xFFFF);
////			HAL_UART_Transmit(&hlpuart1, dist, 3, 0xFFFF);
////		}
//		//		END OF DISTANCE SENSOR READ
//
//		//		START OF IMU READ
//		float dt = (now - lastIMU) / 1000.0f;
//		lastIMU = now;
//
//		// Read sensor data
//		xyz_a = ACCEL_READ_ACCELERATION();
//
//		// Remove ground reference bias
//		xyz_a = vSub(xyz_a, xyz_ground);
//
//		// Apply deadzone to accelerations (in droid frame)
//		float accel_x = applyAccelDeadzone(xyz_a.x, ACCEL_DEADZONE);
//		float accel_y = applyAccelDeadzone(xyz_a.y, ACCEL_DEADZONE);
//
//		// Accumulate error for integral term
//		error_x += accel_x * dt;
//		error_y += accel_y * dt;
//
//		// Calculate derivative of acceleration (jerk)
//		float accel_dot_x = (accel_x - prev_accel_x) / dt;
//		float accel_dot_y = (accel_y - prev_accel_y) / dt;
//		prev_accel_x = accel_x;
//		prev_accel_y = accel_y;
//
//		// PID control: directly counteract acceleration in droid frame
//		// Negative sign because we want to oppose the acceleration
//		Fx_d = -(KP * accel_x + KI * error_x + KD * accel_dot_x);
//		Fy_d = -(KP * accel_y + KI * error_y + KD * accel_dot_y);
//
//		// Clamp forces to maximum
//		float F_total = sqrtf(Fx_d * Fx_d + Fy_d * Fy_d);
//		if (F_total > MAX_FORCE) {
//			float scale = MAX_FORCE / F_total;
//			Fx_d *= scale;
//			Fy_d *= scale;
//		}
//
//		// Assign motor values and set PWM
//		assignMotorValues(Fx_d, Fy_d, &m1, &m2, &m3, &m4);
//		TIM_SetPWM(m1, m2, m3, m4);
//
//		//		END OF IMU READ
//
//		HAL_Delay(5);
		TIM_SetPWM(5.5, 5.5, 5.5, 5.5);  // Constant low throttle
			    HAL_Delay(10);
	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_2;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00000509;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00000509;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void) {

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
	if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = { 0 };

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
	hspi1.Init.MasterInterDataIdleness =
	SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
	hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
	HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection =
	SPI_GRP1_GPDMA_CH0_TCF_TRG;
	HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity =
	SPI_TRIG_POLARITY_RISING;
	if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1,
			&HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = { 0 };

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 0x7;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi3.Init.MasterInterDataIdleness =
	SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
	hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		Error_Handler();
	}
	HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
	HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection =
	SPI_GRP2_LPDMA_CH0_TCF_TRG;
	HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity =
	SPI_TRIG_POLARITY_RISING;
	if (HAL_SPIEx_SetConfigAutonomousMode(&hspi3,
			&HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 3;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 20000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 3;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 20000;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_7,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	/*Configure GPIO pins : PC14 PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PH0 PH1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PC6 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == LPUART1) {
		if (xb[0] == 'C') { // PID tuning
			int dir = (xb[1]) & 1;
			if (xb[2] == 'P') {
				if (dir) {
					KP += 0.1;
				} else {
					KP -= 0.1;
				}
			} else if (xb[1] == 'I') {
				if (dir) {
					KI += 0.01;
				} else {
					KI -= 0.01;
				}
			} else if (xb[1] == 'D') {
				if (dir) {
					KD += 0.01;
				} else {
					KI -= 0.01;
				}
			}
		} else if (xb[0] == 'D') {
			recalibrate = 1;
		}

		// Re-enable interrupt for next character
		HAL_UART_Receive_IT(&hlpuart1, &xb, 3);
	}
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&hlpuart1, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
