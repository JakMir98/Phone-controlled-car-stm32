/*
 * carHandler.c
 *
 *  Created on: Jan 20, 2021
 *      Author: Admin
 */

#include "carHandler.h"

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
	HAL_Delay(400);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	state = RESTING;
}
void LightsOn()
{
	HAL_GPIO_WritePin(LIGHTS_BACK_GPIO_Port, LIGHTS_BACK_Pin, GPIO_PIN_SET);
	TIM1->CCR3 = 10;
}
void LightsOff()
{
	HAL_GPIO_WritePin(LIGHTS_BACK_GPIO_Port, LIGHTS_BACK_Pin, GPIO_PIN_RESET);
	TIM1->CCR3 = 0;
}

void AdjustBrightness()
{
	if (lightsBrightness == 0) TIM1->CCR3 = 20;
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
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	forwardCounter = 0;
	backwardCounter = 0;
	rightTurningCounter = 0;
	leftTurningCounter = 0;
	isLeftLightActive = 0;
	isRightLightActive = 0;
}
void MotorsReset()
{
	HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN3_GPIO_Port, DC_IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DC_IN4_GPIO_Port, DC_IN4_Pin, GPIO_PIN_RESET);
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	state = RESTING;
}
void WaitForAction()
{
	state = RESTING;
	MotorsReset();
	//__WFI();
}


