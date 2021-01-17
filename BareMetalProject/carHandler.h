/**************************************************************************************
 * This file is a part of Remote Controlled Car Bare Metal Project for TMP2 *
 **************************************************************************************/
 
#ifndef carHandler_h
#define carHandler_h

#include "stm32f411xe.h"
#include "system_stm32f4xx.h"
#include "gpio.h"

/**************************************************************************************\
* Private definitions
\**************************************************************************************/
#define SPEED_MAX_LEVEL 										3
#define TURNING_MAX_LEVEL 									1
#define MOTOR_SPEED_MIN 										25
#define MOTOR_SPEED_MEDIUM 						50
#define MOTOR_SPEED_MORE_MEDIUM 	75
#define MOTOR_SPEED_MAX 									100

/**************************************************************************************\
* Private enums
\**************************************************************************************/
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
void delayMs(int delay);
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

#endif
