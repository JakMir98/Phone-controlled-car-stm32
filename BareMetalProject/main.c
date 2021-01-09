#include "stm32f411xe.h"
#include "system_stm32f4xx.h"
#include "systemClock.h"
#include "gpio.h"
#include "uart.h"
#include "pwm.h"

// constants define
#define SPEED_MAX_LEVEL 										3
#define TURNING_MAX_LEVEL 									1
#define MOTOR_SPEED_MIN 										25
#define MOTOR_SPEED_MEDIUM 						50
#define MOTOR_SPEED_MORE_MEDIUM 	75
#define MOTOR_SPEED_MAX 									100

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

// VARIABLES
static State state = RESTING;
//uart handling
uint8_t static received;

// speed counters
static uint8_t forwardCounter = 0;
static uint8_t backwardCounter = 0;
static uint8_t rightTurningCounter = 0;
static uint8_t leftTurningCounter = 0;
static uint8_t lightsBrightness = 0;

// light ticks handle
static  uint8_t isRightLightActive=0;
static  uint8_t isLeftLightActive=0;

extern uint32_t SystemCoreClock;
// FUNCTION PROTOTYPES

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
void RightLightTickHandle(void);
void LeftLightTickHandle(void);
void WaitForAction(void);
void MotorsReset(void);

void delayMs(int delay);
void USART2_IRQHandler(void);

int main(void)
{
	SystemInit();
	//SYS_CLOCK_INIT();
	GPIO_INIT();
	//PWM_INIT();
	
	while(1)
	{
		if (ReadPinOnGPIOC(USER_BUTTON)) // if PC13 is high
 		{
			ResetPinOnGPIOA(ON_BOARD_LED_PIN);
 		}
 		else
 		{
			SetPinOnGPIOA(ON_BOARD_LED_PIN);
 		}
		
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

	  if(isRightLightActive == 1)
	  {
		  RightLightTickHandle();
		  //HAL_GPIO_WritePin(ON_BOARD_LED_GPIO_Port, ON_BOARD_LED_Pin, GPIO_PIN_RESET);
	  }
	  else if(isRightLightActive == 0) ;//HAL_GPIO_WritePin(LIGHTS_FRONT_GPIO_Port, LIGHTS_FRONT_Pin, GPIO_PIN_RESET);

	  if(isLeftLightActive == 1)
	  {
		  LeftLightTickHandle();
		  //HAL_GPIO_WritePin(LIGHTS_FRONT_GPIO_Port, LIGHTS_FRONT_Pin, GPIO_PIN_RESET);
	  }
	  else if(isLeftLightActive == 0) ;//HAL_GPIO_WritePin(ON_BOARD_LED_GPIO_Port, ON_BOARD_LED_Pin, GPIO_PIN_RESET);
	}
}

void delayMs( int n) 
{  
	for(int j = 0; j < 10000*n; j++)
		__NOP();
}
	
void USART2_IRQHandler(void)
{
    // check if the source is read interrupt
    if (USART2->SR & (1 << 5)) {
	
				received = USART2_GetChar();

				if(received == GO_FORWARD_CHAR)
				{
					state = GO_FORWARD;
					USART2_SendString("forward");
					forwardCounter++;
					if (forwardCounter > SPEED_MAX_LEVEL) forwardCounter = 0;
				}
				else if(received == GO_BACKWARD_CHAR)
				{
					state = GO_BACKWARD;
					backwardCounter++;
					USART2_SendString("backward");
					if (backwardCounter > SPEED_MAX_LEVEL) backwardCounter = 0;
				}
				else if(received == TURN_RIGHT_CHAR )
				{
					state = TURN_RIGHT;
					rightTurningCounter++;
					USART2_SendString("right");
					if (rightTurningCounter > TURNING_MAX_LEVEL) rightTurningCounter = 0;
				}
				else if(received == TURN_LEFT_CHAR )
				{
					state = TURN_LEFT;
					leftTurningCounter++;
					USART2_SendString("left");
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
				else if(received == RESET_CHAR  )
						state = RESETTING;
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
    }
		//USART2->CR1 |= (1 << 7);
		USART2->CR1 |= (1 << 5);
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
	SetPinOnGPIOC(BUZZER_PIN);
	delayMs(250);
	ResetPinOnGPIOC(BUZZER_PIN);
	state = RESTING;
}

void LightsOn()
{
	SetPinOnGPIOA(LIGHTS_BACK_PIN);
	SetPinOnGPIOB(LIGHTS_FRONT_PIN);
	// todo 
	TIM1->CCR3 = 50;
}

void LightsOff()
{
	ResetPinOnGPIOA(LIGHTS_BACK_PIN);
	ResetPinOnGPIOB(LIGHTS_FRONT_PIN);
	// todo
	TIM1->CCR3 = 0;
}

void RightLightTickHandle(void)
{
	// todo spi1
	//HAL_GPIO_TogglePin(LIGHTS_FRONT_GPIO_Port, LIGHTS_FRONT_Pin);
	//HAL_Delay(500);
}

void LeftLightTickHandle(void)
{
	// todo spi2
	//HAL_GPIO_TogglePin(ON_BOARD_LED_GPIO_Port, ON_BOARD_LED_Pin);
	//HAL_Delay(500);
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
	SetPinOnGPIOC(DC_IN_1_PIN);
	ResetPinOnGPIOC(DC_IN_2_PIN);

	TIM1->CCR2 = pwm_value1;
	SetPinOnGPIOC(DC_IN_3_PIN);
	ResetPinOnGPIOC(DC_IN_4_PIN);
}

void MotorsBackward(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	ResetPinOnGPIOC(DC_IN_1_PIN);
	SetPinOnGPIOC(DC_IN_2_PIN);

	TIM1->CCR2 = pwm_value1;
	ResetPinOnGPIOC(DC_IN_3_PIN);
	SetPinOnGPIOC(DC_IN_4_PIN);
}

void MotorsTurningRight(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	SetPinOnGPIOC(DC_IN_1_PIN);
	ResetPinOnGPIOC(DC_IN_2_PIN);

	TIM1->CCR2 = pwm_value1;
	ResetPinOnGPIOC(DC_IN_3_PIN);
	SetPinOnGPIOC(DC_IN_4_PIN);
}

void MotorsTurningLeft(uint8_t pwm_value1)
{
	TIM1->CCR1 = pwm_value1;
	ResetPinOnGPIOC(DC_IN_1_PIN);
	SetPinOnGPIOC(DC_IN_2_PIN);

	TIM1->CCR2 = pwm_value1;
	SetPinOnGPIOC(DC_IN_3_PIN);
	ResetPinOnGPIOC(DC_IN_4_PIN);
}

void Reset()
{
	ResetPinOnGPIOC(DC_IN_1_PIN);
	ResetPinOnGPIOC(DC_IN_2_PIN);
	ResetPinOnGPIOC(DC_IN_3_PIN);
	ResetPinOnGPIOC(DC_IN_4_PIN);
	
	ResetPinOnGPIOA(ON_BOARD_LED_PIN);
	ResetPinOnGPIOA(LIGHTS_BACK_PIN);
	ResetPinOnGPIOB(LIGHTS_FRONT_PIN);

	forwardCounter = 0;
	backwardCounter = 0;
	rightTurningCounter = 0;
	leftTurningCounter = 0;
	isLeftLightActive = 0;
	isRightLightActive = 0;
}

void MotorsReset()
{
	ResetPinOnGPIOC(DC_IN_1_PIN);
	ResetPinOnGPIOC(DC_IN_2_PIN);
	ResetPinOnGPIOC(DC_IN_3_PIN);
	ResetPinOnGPIOC(DC_IN_4_PIN);
	state = RESTING;
}

void WaitForAction()
{
	state = RESTING;
	//MotorsReset();
	//__WFI();
}













