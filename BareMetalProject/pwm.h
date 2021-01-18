/**************************************************************************************
 * This file is a part of Phone Controlled Car Bare Metal Project for TMP2 *
 **************************************************************************************/

#ifndef pwm_h
#define pwm_h

#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

/**************************************************************************************\
* Private definitions
\**************************************************************************************/
// GPIOA
#define PWM_DC_CHANNEL_1_PIN 8
#define PWM_DC_CHANNEL_2_PIN 9
#define PWM_LED_FRONT_PIN 10

/**************************************************************************************\
* Private prototypes
\**************************************************************************************/

/**
 * @brief PWM initialization, used TIM1, GPIOA ports used 8,9,10
 */
void PWM_INIT(void);

#endif
