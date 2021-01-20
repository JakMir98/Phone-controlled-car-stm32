/*
 * carHandler.h
 *
 *  Created on: Jan 20, 2021
 *      Author: Admin
 */

#ifndef CARHANDLER_H_
#define CARHANDLER_H_

#include "stm32f1xx.h"
#include "system_stm32f1xx.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"



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

/**************************************************************************************\
* Private variables
\**************************************************************************************/
// speed counters
extern uint8_t forwardCounter;
extern uint8_t backwardCounter;
extern uint8_t rightTurningCounter;
extern uint8_t leftTurningCounter;
extern uint8_t lightsBrightness;

// light ticks handle
extern  uint8_t isRightLightActive;
extern  uint8_t isLeftLightActive;
extern  uint8_t isRightOn;
extern  uint8_t isLeftOn;

extern volatile State state;
/**************************************************************************************\
* Private prototypes
\**************************************************************************************/
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
void WaitForAction(void);
void MotorsReset(void);
#endif /* CARHANDLER_H_ */
