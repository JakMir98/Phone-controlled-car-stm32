/**************************************************************************************
 * This file is a part of Phone Controlled Car Bare Metal Project for TMP2 *
 **************************************************************************************/
 
#ifndef gpio_h
#define gpio_h

#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

/**************************************************************************************\
* Private definitions
\**************************************************************************************/
// GPIOA
#define LIGHTS_BACK_PIN 7
#define ON_BOARD_LED_PIN 5

// GPIOB
#define LIGHTS_FRONT_PIN 6
#define TRIG_PIN 5	
#define ECHO_PIN 4

// GPIOC
#define BUZZER_PIN 7
#define DC_IN_1_PIN 2
#define DC_IN_2_PIN 3
#define DC_IN_3_PIN 0
#define DC_IN_4_PIN 1
#define RIGHT_LIGHT_PIN 11
#define LEFT_LIGHT_PIN 10
#define USER_BUTTON 13

/**************************************************************************************\
* Global macros
\**************************************************************************************/
#define SetPin(port, pinNum) (port->BSRR |= (1<<pinNum))
#define ResetPin(port, pinNum) (port->BSRR |= (1<<(16+pinNum)))
#define ReadPin(port, pinNum) (port->IDR & (1<<pinNum))
#define Mask(position) (1UL << position)


/**************************************************************************************\
* Private prototypes
\**************************************************************************************/

/**
 * @brief All used GPIO initialization, outputs and inputs, set fast mode, user button interrupt enable
 */
void GPIO_INIT(void);

/**
 * @brief  User button interrupt handler
 */
void EXTI15_10_IRQHandler(void);

#endif
