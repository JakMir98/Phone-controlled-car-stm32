#include "stm32f411xe.h"
#include "system_stm32f4xx.h"
#include "gpio.h"
#include "uart.h"
#include "pwm.h"
#include "spi.h"
#include "periodicTimer.h"
#include "carHandler.h"

/**************************************************************************************\
* Private variables
\**************************************************************************************/
static uint8_t received; // USART handling var
extern uint32_t SystemCoreClock;

uint8_t forwardCounter = 0;
uint8_t backwardCounter = 0;
uint8_t rightTurningCounter = 0;
uint8_t leftTurningCounter = 0;
uint8_t lightsBrightness = 0;

// light ticks handle
uint8_t isRightLightActive = 0;
uint8_t isLeftLightActive = 0;
uint8_t isRightOn = 0;
uint8_t isLeftOn = 0;

volatile State state = RESETTING; // FSM

/**************************************************************************************\
* Private prototypes
\**************************************************************************************/
void USART2_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void);

/**************************************************************************************\
* Program start
\**************************************************************************************/
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

/**************************************************************************************\
* Private functions
\**************************************************************************************/
/**
 * @brief USART2 interrupt handle function
 * Read char from usart and set state
 */
void USART2_IRQHandler(void)
{
	// check if the source is read interrupt
	if (USART2->SR & (1 << 5)) 
	{
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
			if (lightsBrightness > TURNING_MAX_LEVEL) lightsBrightness = 0;
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

/**
 * @brief TIM10 interrupt handle function
 * every 1 second run timer and handle turn signal 
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
	if (TIM10->DIER & 0x01) 
	{
		if (TIM10->SR & 0x01) 
		{
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

