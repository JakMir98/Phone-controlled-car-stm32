#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

// GPIOA
#define LIGHTS_BACK_PIN 												7
#define ON_BOARD_LED_PIN 										5

// GPIOB
#define LIGHTS_FRONT_PIN 											6
#define TRIG_PIN 																			5	
#define ECHO_PIN 																		4

// GPIOC
#define BUZZER_PIN 																7
#define DC_IN_1_PIN 																2
#define DC_IN_2_PIN 																3
#define DC_IN_3_PIN 																0
#define DC_IN_4_PIN 																1
#define RIGHT_LIGHT_PIN 												11
#define LEFT_LIGHT_PIN 													10
#define USER_BUTTON															13

#define SetPinOnGPIOA(pinNum) (GPIOA->BSRR |= (1<<pinNum))
#define SetPinOnGPIOB(pinNum) (GPIOB->BSRR |= (1<<pinNum))
#define SetPinOnGPIOC(pinNum) (GPIOC->BSRR |= (1<<pinNum))

#define ResetPinOnGPIOA(pinNum) (GPIOA->BSRR |= (1<<(16+pinNum)))
#define ResetPinOnGPIOB(pinNum) (GPIOB->BSRR |= (1<<(16+pinNum)))
#define ResetPinOnGPIOC(pinNum) (GPIOC->BSRR |= (1<<(16+pinNum)))

#define ReadPinOnGPIOA(pinNum) (GPIOA->IDR & (1<<pinNum))
#define ReadPinOnGPIOB(pinNum) (GPIOB->IDR & (1<<pinNum))
#define ReadPinOnGPIOC(pinNum) (GPIOC->IDR & (1<<pinNum))

void GPIO_INIT(void);
