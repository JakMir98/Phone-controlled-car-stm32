#include "stm32f411xe.h"
#include "system_stm32f4xx.h"
#include "gpio.h"
#include "uart.h"
#include "pwm.h"
#include "spi.h"
#include "periodicTimer.h"

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
static volatile State state = RESETTING;
//uart handling
static uint8_t received;

// speed counters
static uint8_t forwardCounter = 				0;
static uint8_t backwardCounter = 		0;
static uint8_t rightTurningCounter = 0;
static uint8_t leftTurningCounter = 	0;
static uint8_t lightsBrightness = 			0;

// light ticks handle
static  uint8_t isRightLightActive = 	0;
static  uint8_t isLeftLightActive = 		0;
static  uint8_t isRightOn = 	0;
static  uint8_t isLeftOn = 		0;
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
	GPIO_INIT();
	USART_INIT();
	PWM_INIT();
	SPI_INIT();
	PERIODIC_TIMER_INIT();
	
	while(1)
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
				{
					USART2_SendString("horn");
					state = HORN;
				}	
				else if(received == ADJUST_BRIGHTNESS_CHAR )
				{
					state = FRONT_LIGHTS_BRIGHTNESS_CHANGE;
					lightsBrightness++;
					USART2_SendString("adjust");
					if (lightsBrightness > SPEED_MAX_LEVEL) lightsBrightness = 0;
				}
				else if(received == LIGHTS_ON_CHAR  )
				{
					USART2_SendString("lights on");
					state = LIGHTS_ON;
				}
				else if(received == LIGHTS_OFF_CHAR  )
				{
					USART2_SendString("lights off");
					state = LIGHTS_OFF;
				}
				else if(received == DO_NOTHING_CHAR  )
				{
					USART2_SendString("nothing");
					state = RESTING;
				}
				else if(received == RESET_CHAR  )
				{
					USART2_SendString("reset\n");
						state = RESETTING;
				}
				else if(received == LEFT_LIGHT_CHAR)
				{
					USART2_SendString("left light");
					isRightLightActive = 0;
					if (isLeftLightActive == 0) isLeftLightActive = 1;
					else isLeftLightActive = 0;
				}
				else if(received == RIGHT_LIGHT_CHAR  )
				{
					USART2_SendString("right light");
					isLeftLightActive = 0;
					if (isRightLightActive == 0) isRightLightActive = 1;
					else isRightLightActive = 0;
				}
    }
		//USART2->CR1 |= (1 << 7); transmiting interrupt not needed
		USART2->CR1 |= (1 << 5);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	 if (TIM10->DIER & 0x01) {
        if (TIM10->SR & 0x01) {
            TIM10->SR &= ~(1U << 0);
        }
    }
			 if(isRightOn == 0) isRightOn = 1;
			 else isRightOn = 	0;
		
			if(isLeftOn == 0) isLeftOn = 1;
			else isLeftOn = 	0;
		
			if (isRightLightActive == 1 && isRightOn == 1) SetPin(GPIOC, RIGHT_LIGHT_PIN);
		  else if (isRightLightActive == 1 && isRightOn == 0) ResetPin(GPIOC, RIGHT_LIGHT_PIN);
		  else if (isLeftLightActive == 1 && isLeftOn == 1) SetPin(GPIOC, LEFT_LIGHT_PIN);
		  else if (isLeftLightActive == 1 && isLeftOn == 0) ResetPin(GPIOC, LEFT_LIGHT_PIN);
		
		  if (isRightLightActive == 0) ResetPin(GPIOC, RIGHT_LIGHT_PIN);
			else if(isLeftLightActive == 0) ResetPin(GPIOC, LEFT_LIGHT_PIN);
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
	SetPin(GPIOB, LIGHTS_FRONT_PIN);
	TIM1->CCR3 = 50;
}

void LightsOff()
{
	ResetPin(GPIOA, LIGHTS_BACK_PIN);
	ResetPin(GPIOB, LIGHTS_FRONT_PIN);
	TIM1->CCR3 = 0;
}

void RightLightTickHandle(void)
{
	
}

void LeftLightTickHandle(void)
{
	
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
	ResetPin(GPIOC, DC_IN_1_PIN);
	ResetPin(GPIOC, DC_IN_2_PIN);
	ResetPin(GPIOC, DC_IN_3_PIN);
	ResetPin(GPIOC, DC_IN_4_PIN);
	
	ResetPin(GPIOA, ON_BOARD_LED_PIN);
	ResetPin(GPIOA, LIGHTS_BACK_PIN);
	ResetPin(GPIOB, LIGHTS_FRONT_PIN);

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













