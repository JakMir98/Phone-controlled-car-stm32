/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//FSM
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
	RESETTING,
	RESTING
} State;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//code for SPI handling
#define MCP_IODIR 		0x00
#define MCP_IPOL 		0x01
#define MCP_GPINTEN		0x02
#define MCP_DEFVAL 		0x03
#define MCP_INTCON 		0x04
#define MCP_IOCON 		0x05
#define MCP_GPPU 		0x06
#define MCP_INTF 		0x07
#define MCP_INTCAP 		0x08
#define MCP_GPIO 		0x09
#define MCP_OLAT 		0x0A

//UART handling
#define GO_FORWARD_CHAR 	'u'
#define GO_BACKWARD_CHAR 	'd'
#define TURN_RIGHT_CHAR 	'r'
#define TURN_LEFT_CHAR 		'l'
#define HORN_CHAR 			'h'
#define ADJUST_BRIGHTNESS_CHAR 'a'
#define LIGHTS_ON_CHAR 		'o'
#define LIGHTS_OFF_CHAR 	'f'
#define DO_NOTHING_CHAR 	'n'
#define RESET_CHAR 			'x'
#define RIGHT_LIGHT_CHAR 	'b'
#define LEFT_LIGHT_CHAR  	'v'

//usefull constants
#define SPEED_MAX_LEVEL 3
#define TURNING_MAX_LEVEL 1
#define MOTOR_SPEED_MIN 25
#define MOTOR_SPEED_MEDIUM 50
#define MOTOR_SPEED_MADMAX 75
#define MOTOR_SPEED_MAX 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//UART variable
//uint8_t buffer[10];
uint8_t received;

//HC-sr04 future variables
//uint32_t local_time, sensor_time;
//uint32_t distance;

uint8_t forwardCounter = 0;
uint8_t backwardCounter = 0;
uint8_t rightTurningCounter = 0;
uint8_t leftTurningCounter = 0;
uint8_t lightsBrightness = 0;

// light ticks handle
uint8_t isRightLightActive=0;
uint8_t isLeftLightActive=0;

uint8_t isRightOn =0;
uint8_t isLeftOn =0;

State state = RESTING;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//HC-sr04
//void delay_us (uint16_t);
//uint32_t HCSR04_READ(void);

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
void RightLight(void); //todo
void LeftLight(void);  //todo
void WaitForAction(void);
void MotorsReset(uint32_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//SPI
void MCP_Write_Reg(uint8_t addr, uint8_t value)
{
	uint8_t tx_buf[] = {0x40, addr, value};

	GPIOA->ODR &= ~GPIO_ODR_ODR0;

	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = tx_buf[0];
	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = tx_buf[1];
	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = tx_buf[2];

	GPIOA->ODR |= GPIO_ODR_ODR0;
}

void Andzej_SPI_Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;

	GPIOA->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE5_0 | GPIO_CRL_MODE7_0;
	GPIOA->CRL &= ~(GPIO_CRL_CNF0_0 | GPIO_CRL_CNF5_0 | GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0);
	GPIOA->CRL |= GPIO_CRL_CNF5_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;

	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(isRightOn == 0)
			isRightOn = 1;
		else isRightOn = 0;
		if(isLeftOn == 0)
			isLeftOn = 1;
		else isLeftOn = 0;

		if(isRightLightActive == 1)
		{
			RightLight();
		}
	    else if(isRightLightActive == 0)
	    {
		  MCP_Write_Reg(MCP_OLAT, 0x00); // both off
	    }

		  if(isLeftLightActive == 1)
		  {
			 LeftLight();
		  }
		  else if((isLeftLightActive == 0 ) & ( isRightLightActive == 0)) {
			 MCP_Write_Reg(MCP_OLAT, 0x00); // both off
		  }
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

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
	else if(received == RESET_CHAR)
		{
			state = RESETTING;
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
	else if(received == LEFT_LIGHT_CHAR)
		{
			isRightLightActive = 0;
			if (isLeftLightActive == 0) isLeftLightActive = 1;
			else isLeftLightActive = 0;
		}
	else if(received == RIGHT_LIGHT_CHAR  )
		{
			isLeftLightActive = 0;
			if (isRightLightActive == 0) isRightLightActive = 1;
			else isRightLightActive = 0;
		}
	HAL_UART_Receive_IT(&huart2, &received, 1); // Ponowne włączenie nasłuchiwania
}
void GoForward()
{
	if (forwardCounter == 0) MotorsForward(MOTOR_SPEED_MIN);
	else if (forwardCounter == 1) MotorsForward(MOTOR_SPEED_MEDIUM);
	else if (forwardCounter == 2) MotorsForward(MOTOR_SPEED_MADMAX);
	else MotorsForward(MOTOR_SPEED_MAX);

}
void GoBackward()
{
	if (backwardCounter == 0) MotorsBackward(MOTOR_SPEED_MIN);
	else if (backwardCounter == 1) MotorsBackward(MOTOR_SPEED_MEDIUM);
	else if (backwardCounter == 2) MotorsBackward(MOTOR_SPEED_MADMAX);
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
void RightLight()
{
	 //dioda SPI 1
	if (isRightOn==1)
		MCP_Write_Reg(MCP_OLAT, 0x01);//right turn on
	else
		MCP_Write_Reg(MCP_OLAT, 0x00);
}

void LeftLight()
{
	//dioda SPI 2
	if (isLeftOn==1)
		MCP_Write_Reg(MCP_OLAT, 0x02); //left turn on
	else
		MCP_Write_Reg(MCP_OLAT, 0x00);

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
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

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

	HAL_GPIO_WritePin(LIGHTS_BACK_GPIO_Port, LIGHTS_BACK_Pin, GPIO_PIN_RESET);
	MCP_Write_Reg(MCP_OLAT, 0x00);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LIGHTS_FRONT_GPIO_Port, LIGHTS_FRONT_Pin, GPIO_PIN_RESET);

	forwardCounter = 0;
	backwardCounter = 0;
	rightTurningCounter = 0;
	leftTurningCounter = 0;
	isLeftLightActive = 0;
	isRightLightActive = 0;
}
void MotorsReset(uint32_t delay)
{
	HAL_Delay(delay);
	HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN4_GPIO_Port, DC_IN4_Pin, GPIO_PIN_RESET);
	state = RESTING;
}
void WaitForAction()
{
	state = RESTING;
	MotorsReset(0);
	__WFI();
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  Andzej_SPI_Init();
  MCP_Write_Reg(MCP_IODIR, ~0x03);
  HAL_UART_Receive_IT(&huart2, &received, 1);

  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  switch(state)
	 	  {
	  	  	  case RESTING:
	  			  WaitForAction();
	  			  break;
	  		  case RESETTING:
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
