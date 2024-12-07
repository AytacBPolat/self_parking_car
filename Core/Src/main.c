/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
#include <stdbool.h>
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t myTime = 0;
bool isTimerActive = false;

uint16_t measure_distance(GPIO_TypeDef* TRIG_GPIO_Port, uint16_t TRIG_Pin, GPIO_TypeDef* ECHO_GPIO_Port, uint16_t ECHO_Pin, TIM_HandleTypeDef tim) {
	uint32_t pMillis;
	uint32_t value1 = 0;
	uint32_t value2 = 0;
	uint16_t distance  = 0; //cm

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&tim) < 10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
	// wait for the echo pin to go high
	while (!(HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && pMillis + 10 >  HAL_GetTick());
	value1 = __HAL_TIM_GET_COUNTER (&tim);

	pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
	// wait for the echo pin to go low
	while ((HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && pMillis + 50 > HAL_GetTick());
	value2 = __HAL_TIM_GET_COUNTER (&tim);

	distance = (value2 - value1) * 0.034 / 2;
	HAL_Delay(50);

	return distance;
}

// CAR MOVEMENT
void goForward() {
	HAL_GPIO_WritePin(RIGHT_MOTOR_POS_GPIO_Port, RIGHT_MOTOR_POS_Pin, 1);
	HAL_GPIO_WritePin(RIGHT_MOTOR_NEG_GPIO_Port, RIGHT_MOTOR_NEG_Pin, 0);
	HAL_GPIO_WritePin(LEFT_MOTOR_POS_GPIO_Port, LEFT_MOTOR_POS_Pin, 1);
	HAL_GPIO_WritePin(LEFT_MOTOR_NEG_GPIO_Port, LEFT_MOTOR_NEG_Pin, 0);
}

void goBackwards() {
	HAL_GPIO_WritePin(RIGHT_MOTOR_POS_GPIO_Port, RIGHT_MOTOR_POS_Pin, 0);
	HAL_GPIO_WritePin(RIGHT_MOTOR_NEG_GPIO_Port, RIGHT_MOTOR_NEG_Pin, 1);
	HAL_GPIO_WritePin(LEFT_MOTOR_POS_GPIO_Port, LEFT_MOTOR_POS_Pin, 0);
	HAL_GPIO_WritePin(LEFT_MOTOR_NEG_GPIO_Port, LEFT_MOTOR_NEG_Pin, 1);
}

void turnRight() {
	HAL_GPIO_WritePin(RIGHT_MOTOR_POS_GPIO_Port, RIGHT_MOTOR_POS_Pin, 1);
	HAL_GPIO_WritePin(RIGHT_MOTOR_NEG_GPIO_Port, RIGHT_MOTOR_NEG_Pin, 0);
	HAL_GPIO_WritePin(LEFT_MOTOR_POS_GPIO_Port, LEFT_MOTOR_POS_Pin, 0);
	HAL_GPIO_WritePin(LEFT_MOTOR_NEG_GPIO_Port, LEFT_MOTOR_NEG_Pin, 1);
}

void turnLeft() {
	HAL_GPIO_WritePin(RIGHT_MOTOR_POS_GPIO_Port, RIGHT_MOTOR_POS_Pin, 0);
	HAL_GPIO_WritePin(RIGHT_MOTOR_NEG_GPIO_Port, RIGHT_MOTOR_NEG_Pin, 1);
	HAL_GPIO_WritePin(LEFT_MOTOR_POS_GPIO_Port, LEFT_MOTOR_POS_Pin, 1);
	HAL_GPIO_WritePin(LEFT_MOTOR_NEG_GPIO_Port, LEFT_MOTOR_NEG_Pin, 0);
}

void stop() {
	HAL_GPIO_WritePin(RIGHT_MOTOR_POS_GPIO_Port, RIGHT_MOTOR_POS_Pin, 0);
	HAL_GPIO_WritePin(RIGHT_MOTOR_NEG_GPIO_Port, RIGHT_MOTOR_NEG_Pin, 0);
	HAL_GPIO_WritePin(LEFT_MOTOR_POS_GPIO_Port, LEFT_MOTOR_POS_Pin, 0);
	HAL_GPIO_WritePin(LEFT_MOTOR_NEG_GPIO_Port, LEFT_MOTOR_NEG_Pin, 0);
}

void reset_time() {
	myTime = 0;
}

void start_timer() {
	isTimerActive = true;
}

void stop_timer() {
	isTimerActive = false;
}

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(SIDE_TRIG_GPIO_Port, SIDE_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

  uint16_t sideDistanceReference = measure_distance(SIDE_TRIG_GPIO_Port, SIDE_TRIG_Pin, SIDE_ECHO_GPIO_Port, SIDE_ECHO_Pin, htim1);

  uint32_t timeDistance = 0;

  bool isParkDone = false;
  bool isParking = false;
  bool isDetectParkingSpot = false;

  bool isStartTurningLeftTimeStarted = false;
  bool isStartTurningLeft = false;
  bool isFinishTurningLeft = false;

  bool isStartTurningRightTimeStarted = false;
  bool isStartTurningRight = false;
  bool isFinishTurningRight = false;

  bool isCarAlignStarted = false;
  bool isCarAligned = false;

  uint32_t carRotateTime = 726;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (!isParkDone) {
		  if (!isParking) {
			  goForward();
			  uint16_t sideDist = measure_distance(SIDE_TRIG_GPIO_Port, SIDE_TRIG_Pin, SIDE_ECHO_GPIO_Port, SIDE_ECHO_Pin, htim1);

			  // calculate gap
			  uint16_t sideDistanceGap = sideDist - sideDistanceReference;
			  // check is there enough gap for parking
			  if (sideDistanceGap >= 10 && sideDistanceGap < 10000 && !isDetectParkingSpot) {
				  isDetectParkingSpot = true;
				  start_timer();
			  }

			  if (isDetectParkingSpot) {
				  if (sideDistanceGap <= 7 || sideDistanceGap >= 65000) {
					  stop_timer();
					  if (myTime >= 300) {
						  stop();
						  timeDistance = myTime;
						  isParking = true;
						  HAL_Delay(1000);
						  reset_time();
						  start_timer();
					  }
					  else {
						  isDetectParkingSpot = false;
						  reset_time();
					  }
				  }
			  }

		  }
		  else {
			  goBackwards();
			  if (myTime >= timeDistance && (!isStartTurningLeft || !isStartTurningRight)) {
				  stop();
				  stop_timer();
				  reset_time();
				  HAL_Delay(1000);
				  isStartTurningLeft = true;
			  }
			  if (!isStartTurningLeftTimeStarted && isStartTurningLeft) {
				  start_timer();
				  isStartTurningLeftTimeStarted = true;
			  }
			  if (isStartTurningLeft && !isFinishTurningLeft) {
				  turnLeft();
				  if (myTime >= carRotateTime) {
					  isStartTurningLeft = false;
					  isFinishTurningLeft = true;
					  stop();
					  stop_timer();
					  reset_time();
					  isCarAlignStarted = true;
				  }
			  }
			  if (isCarAlignStarted && !isCarAligned) {
				  goBackwards();

				  uint16_t frontDist = measure_distance(FRONT_TRIG_GPIO_Port, FRONT_TRIG_Pin, FRONT_ECHO_GPIO_Port, FRONT_ECHO_Pin, htim1);

				  if (frontDist <= 8) {
					  isCarAligned = true;
					  isCarAlignStarted = false;
					  stop();

					  HAL_Delay(1000);
					  isStartTurningRight = true;
				  }
			  }
			  if (isStartTurningRight && !isStartTurningRightTimeStarted) {
				  start_timer();
				  isStartTurningRightTimeStarted = true;
			  }
			  if (isStartTurningRight && !isFinishTurningRight) {
				  turnRight();
				  if (myTime >= carRotateTime) {
					  isStartTurningRight = false;
					  isFinishTurningRight = true;
					  stop();
					  stop_timer();
					  reset_time();

					  isParkDone = true;
				  }
			  }
			  /*
			  if (!isParkingStopWatchStarted) {
				  stopwatch_start();
				  isParkingStopWatchStarted = true;
			  }
			  if (!isCarAligned) {
				  goBackwards();
				  uint32_t time = stopwatch_read();
				  if (time >= timeDistance / 2) {
					  stopwatch_stop();
					  stopwatch_reset();
					  isCarAligned = true;
				  }
			  }
			  else if (isCarAligned) {
				  if (!isCarTurnLeftStopWatchStarted) {
					  stopwatch_start();
					  isCarTurnLeftStopWatchStarted = true;
				  }
				  uint32_t time = stopwatch_read();
				  turnLeft();
				  if (time >= 250) {
					  stop();
				  }
			  }
			  */

		  }
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FRONT_TRIG_Pin|SIDE_TRIG_Pin|RIGHT_MOTOR_POS_Pin|RIGHT_MOTOR_NEG_Pin
                          |LEFT_MOTOR_NEG_Pin|LEFT_MOTOR_POS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FRONT_TRIG_Pin SIDE_TRIG_Pin RIGHT_MOTOR_POS_Pin RIGHT_MOTOR_NEG_Pin
                           LEFT_MOTOR_NEG_Pin LEFT_MOTOR_POS_Pin */
  GPIO_InitStruct.Pin = FRONT_TRIG_Pin|SIDE_TRIG_Pin|RIGHT_MOTOR_POS_Pin|RIGHT_MOTOR_NEG_Pin
                          |LEFT_MOTOR_NEG_Pin|LEFT_MOTOR_POS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FRONT_ECHO_Pin SIDE_ECHO_Pin */
  GPIO_InitStruct.Pin = FRONT_ECHO_Pin|SIDE_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3 && isTimerActive) {
    	myTime++;
    }
}

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
