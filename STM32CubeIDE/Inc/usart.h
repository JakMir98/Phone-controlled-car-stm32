/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32f1xx_it.h"
#include "stm32f1xx.h"
#include "system_stm32f1xx.h"
#include "tim.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */


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
#define MOTOR_SPEED_MIN 65
#define MOTOR_SPEED_MEDIUM 75
#define MOTOR_SPEED_MADMAX 90
#define MOTOR_SPEED_MAX 100


/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
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
void MotorsReset();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
