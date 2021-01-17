#include "carHandler.h"

/**
 * @brief Simple delay function
 *
 * @param miliseconds to wait.
 */
void delayMs( int n) 
{  
	for(int j = 0; j < 10000*n; j++)
		__NOP();
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
	SetPin(GPIOC, BUZZER_PIN);
	delayMs(250);
	ResetPin(GPIOC, BUZZER_PIN);
	state = RESTING;
}

void LightsOn()
{
	SetPin(GPIOA, LIGHTS_BACK_PIN);
	//SetPin(GPIOB, LIGHTS_FRONT_PIN);
	TIM1->CCR3 = 25;
}

void LightsOff()
{
	ResetPin(GPIOA, LIGHTS_BACK_PIN);
	//ResetPin(GPIOB, LIGHTS_FRONT_PIN);
	TIM1->CCR3 = 0;
}

void AdjustBrightness()
{
	if (lightsBrightness == 0) TIM1->CCR3 = 25;
	else TIM1->CCR3 = 100;
}

void MotorsForward(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	SetPin(GPIOC, DC_IN_1_PIN);
	ResetPin(GPIOC, DC_IN_2_PIN);
	
	TIM1->CCR2 = pwm_value1;
	SetPin(GPIOC, DC_IN_3_PIN);
	ResetPin(GPIOC, DC_IN_4_PIN);
}

void MotorsBackward(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	ResetPin(GPIOC, DC_IN_1_PIN);
	SetPin(GPIOC, DC_IN_2_PIN);

	TIM1->CCR2 = pwm_value1;
	ResetPin(GPIOC, DC_IN_3_PIN);
	SetPin(GPIOC, DC_IN_4_PIN);
}

void MotorsTurningRight(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	SetPin(GPIOC, DC_IN_1_PIN);
	ResetPin(GPIOC, DC_IN_2_PIN);

	TIM1->CCR2 = pwm_value1;
	ResetPin(GPIOC, DC_IN_3_PIN);
	SetPin(GPIOC, DC_IN_4_PIN);
}

void MotorsTurningLeft(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	ResetPin(GPIOC, DC_IN_1_PIN);
	SetPin(GPIOC, DC_IN_2_PIN);

	TIM1->CCR2 = pwm_value1;
	SetPin(GPIOC, DC_IN_3_PIN);
	ResetPin(GPIOC, DC_IN_4_PIN);
}

void Reset()
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	
	ResetPin(GPIOC, DC_IN_1_PIN);
	ResetPin(GPIOC, DC_IN_2_PIN);
	ResetPin(GPIOC, DC_IN_3_PIN);
	ResetPin(GPIOC, DC_IN_4_PIN);
	
	ResetPin(GPIOA, ON_BOARD_LED_PIN);
	ResetPin(GPIOA, LIGHTS_BACK_PIN);
	ResetPin(GPIOB, LIGHTS_FRONT_PIN);
	ResetPin(GPIOC, BUZZER_PIN);

	forwardCounter = 0;
	backwardCounter = 0;
	rightTurningCounter = 0;
	leftTurningCounter = 0;
	isLeftLightActive = 0;
	isRightLightActive = 0;
}

void MotorsReset()
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	ResetPin(GPIOC, DC_IN_1_PIN);
	ResetPin(GPIOC, DC_IN_2_PIN);
	ResetPin(GPIOC, DC_IN_3_PIN);
	ResetPin(GPIOC, DC_IN_4_PIN);
}

void WaitForAction()
{
	state = RESTING;
	MotorsReset();
	//__WFI();
}
