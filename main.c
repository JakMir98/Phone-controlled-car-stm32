/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define GO_FORWARD_CHAR 'u'
#define GO_BACKWARD_CHAR 'd'
#define TURN_RIGHT_CHAR 'r'
#define TURN_LEFT_CHAR 'l'
#define HORN_CHAR 'h'
#define ADJUST_BRIGHTNESS_CHAR 'a'
#define LIGHTS_ON_CHAR 'o'
#define LIGHTS_OFF_CHAR 'f'
#define DO_NOTHING_CHAR 'n'

#define SPEED_MAX_LEVEL 3
#define TURNING_MAX_LEVEL 1

#define MOTOR_SPEED_MIN 25
#define MOTOR_SPEED_MEDIUM 50
#define MOTOR_SPEED_MORE_MEDIUM 75
#define MOTOR_SPEED_MAX 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int counter = 1;
uint32_t local_time, sensor_time;
uint32_t distance;
char received;

uint8_t forwardCounter = 0;
uint8_t backwardCounter = 0;
uint8_t rightTurningCounter = 0;
uint8_t leftTurningCounter = 0;
uint8_t lightsBrightness = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void delay_us (uint16_t);
uint32_t HCSR04_READ(void);
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);

void GoForward(void);
void GoBackward(void);
void TurnRight(void);
void TurnLeft(void);
void Horn(void);
void LightsOn(void);
void LightsOff(void);
void AdjustBrightness(void);
void MotorsBackward(uint8_t pwm_value1);
void MotorsForward(uint8_t pwm_value1);
void MotorsTurningRight(uint8_t pwm_value1);
void MotorsTurningLeft(uint8_t pwm_value1);
void Reset(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum
{
	GO_FORWARD,
	GO_BACKWARD,
	TURN_RIGHT,
	TURN_LEFT,
	HORN,
	FRONT_LIGHTS_BRIGHTNESS_CHANGE,
	LIGHTS_ON,
	LIGHTS_OFF,
	RESTING
} State;

State state = RESTING;

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}
uint32_t HCSR04_READ()
{
	local_time=0;
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)));  // wait for the ECHO pin to go high
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))    // while the pin is high
	{
	  local_time++;   // measure time for which the pin is high
	  delay_us(1);
	}
	return local_time;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// testing only
	if (GPIO_Pin == USER_BUTTON_Pin)
	{
		  if (counter == 0)
		  {
			    TIM1->CCR3 = 1;
		  }
		  else if (counter == 1)
		  {
			TIM1->CCR3 = 50;
			HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DC_IN4_GPIO_Port, DC_IN4_Pin, GPIO_PIN_RESET);
		  }
		  else if (counter == 2)TIM1->CCR3 = 75;
		  else if (counter == 3)TIM1->CCR3 = 100;
		  else if (counter == 4)
		  {
			TIM1->CCR3 = 0;
			counter = 0;
			HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN3_Pin, GPIO_PIN_RESET);
		  }
	}

	counter++;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM10)
	{
		// Jeżeli przerwanie pochodzi od timera 10
		static uint16_t cnt = 0; // Licznik wyslanych wiadomosci
		 uint8_t data[50];// Tablica przechowujaca wysylana wiadomosc.
		 uint16_t size = 0; // Rozmiar wysylanej wiadomosci ++cnt; // Zwiekszenie licznika wyslanych wiadomosci.

		 ++cnt; // Zwiekszenie licznika wyslanych wiadomosci.
		 size = sprintf(data, "Odczyt: %d.\n\r", cnt); // Stworzenie wiadomosci do wyslania oraz przypisanie ilosci wysylanych znakow do zmiennej size.
		 HAL_UART_Transmit_IT(&huart2, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
		 //HAL_GPIO_TogglePin(ON_BOARD_LED_GPIO_Port, ON_BOARD_LED_Pin);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t Data[50]; // Tablica przechowujaca wysylana wiadomosc.
	uint16_t size = 0; // Rozmiar wysylanej wiadomosci

	if(received == GO_FORWARD_CHAR)
	{
		state = GO_FORWARD;
		forwardCounter++;
		if (forwardCounter > SPEED_MAX_LEVEL) forwardCounter = 0;
	}
	else if(received == GO_BACKWARD_CHAR)
	{
		state = GO_BACKWARD;
		backwardCounter++;
		if (backwardCounter > SPEED_MAX_LEVEL) backwardCounter = 0;
	}
	else if(received == TURN_RIGHT_CHAR )
	{
		state = TURN_RIGHT;
		rightTurningCounter++;
		if (rightTurningCounter > TURNING_MAX_LEVEL) rightTurningCounter = 0;
	}
	else if(received == TURN_LEFT_CHAR )
	{
		state = TURN_LEFT;
		leftTurningCounter++;
		if (leftTurningCounter > TURNING_MAX_LEVEL) leftTurningCounter = 0;
	}
	else if(received == HORN_CHAR )
		state = HORN;
	else if(received == ADJUST_BRIGHTNESS_CHAR )
	{
		state = FRONT_LIGHTS_BRIGHTNESS_CHANGE;
		lightsBrightness++;
		if (lightsBrightness > SPEED_MAX_LEVEL) lightsBrightness = 0;
	}
	else if(received == LIGHTS_ON_CHAR  )
		state = LIGHTS_ON;
	else if(received == LIGHTS_OFF_CHAR  )
		state = LIGHTS_OFF;
	else if(received == DO_NOTHING_CHAR  )
		state = RESTING;

	HAL_UART_Transmit_IT(&huart2, Data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	HAL_UART_Receive_IT(&huart2, &received, 1); // Ponowne włączenie nasłuchiwania
}
void GoForward()
{
	if (forwardCounter == 0) MotorsForward(MOTOR_SPEED_MIN);
	else if (forwardCounter == 1) MotorsForward(MOTOR_SPEED_MEDIUM);
	else if (forwardCounter == 2) MotorsForward(MOTOR_SPEED_MORE_MEDIUM);
	else MotorsForward(MOTOR_SPEED_MAX);

}
void GoBackward()
{
	if (backwardCounter == 0) MotorsBackward(MOTOR_SPEED_MIN);
	else if (backwardCounter == 1) MotorsBackward(MOTOR_SPEED_MEDIUM);
	else if (backwardCounter == 2) MotorsBackward(MOTOR_SPEED_MORE_MEDIUM);
	else MotorsBackward(MOTOR_SPEED_MAX);
}
void TurnRight()
{
	if (rightTurningCounter == 0) MotorsTurningRight(MOTOR_SPEED_MEDIUM);
	else MotorsTurningRight(MOTOR_SPEED_MAX);

}
void TurnLeft()
{
	if (leftTurningCounter == 0) MotorsTurningLeft(MOTOR_SPEED_MEDIUM);
	else MotorsTurningLeft(MOTOR_SPEED_MAX);
}
void Horn()
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	state = RESTING;
}
void LightsOn()
{
	HAL_GPIO_WritePin(LIGHTS_BACK_GPIO_Port, LIGHTS_BACK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LIGHTS_FRONT_GPIO_Port, LIGHTS_FRONT_Pin, GPIO_PIN_SET);
}
void LightsOff()
{
	HAL_GPIO_WritePin(LIGHTS_BACK_GPIO_Port, LIGHTS_BACK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LIGHTS_FRONT_GPIO_Port, LIGHTS_FRONT_Pin, GPIO_PIN_RESET);
}
void AdjustBrightness()
{
	if (lightsBrightness == 0) TIM1->CCR3 = 10;
	else if (lightsBrightness == 1) TIM1->CCR3 = 25;
	else if (lightsBrightness == 2) TIM1->CCR3 = 50;
	else if (lightsBrightness == 3) TIM1->CCR3 = 75;
	else TIM1->CCR3 = 100;
}
void MotorsForward(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_RESET);

	TIM1->CCR2 = pwm_value1;
	HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DC_IN4_GPIO_Port, DC_IN4_Pin, GPIO_PIN_RESET);
}
void MotorsBackward(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_SET);

	TIM1->CCR2 = pwm_value1;
	HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN4_GPIO_Port, DC_IN4_Pin, GPIO_PIN_SET);
}
void MotorsTurningRight(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_RESET);

	TIM1->CCR2 = pwm_value1;
	HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN4_GPIO_Port, DC_IN4_Pin, GPIO_PIN_SET);
}
void MotorsTurningLeft(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_SET);

	TIM1->CCR2 = pwm_value1;
	HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DC_IN4_GPIO_Port, DC_IN4_Pin, GPIO_PIN_RESET);
}
void Reset()
{
	HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN4_GPIO_Port, DC_IN4_Pin, GPIO_PIN_RESET);

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  //HAL_InitTick(0);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  TIM1->CCR3 = 0;
  HAL_UART_Receive_IT(&huart2, &received, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(state)
	  {
		  case RESTING:
			  Reset();
			  break;
		  case GO_FORWARD:
			  GoForward();
			  break;
		  case GO_BACKWARD:
			  GoBackward();
			  break;
		  case TURN_RIGHT:
			  TurnRight();
			  break;
		  case TURN_LEFT:
			  TurnLeft();
			  break;
		  case HORN:
			  Horn();
			  break;
		  case FRONT_LIGHTS_BRIGHTNESS_CHANGE:
			  AdjustBrightness();
			  break;
		  case LIGHTS_ON:
			  LightsOn();
			  break;
		  case LIGHTS_OFF:
			  LightsOff();
			  break;
	  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 639;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 81;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DC_IN3_Pin|DC_IN4_Pin|DC_IN1_Pin|DC_IN2_Pin
                          |BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ON_BOARD_LED_Pin|LIGHTS_BACK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIG_Pin|LIGHTS_FRONT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_IN3_Pin DC_IN4_Pin DC_IN1_Pin DC_IN2_Pin
                           BUZZER_Pin */
  GPIO_InitStruct.Pin = DC_IN3_Pin|DC_IN4_Pin|DC_IN1_Pin|DC_IN2_Pin
                          |BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ON_BOARD_LED_Pin LIGHTS_BACK_Pin */
  GPIO_InitStruct.Pin = ON_BOARD_LED_Pin|LIGHTS_BACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin LIGHTS_FRONT_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|LIGHTS_FRONT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
