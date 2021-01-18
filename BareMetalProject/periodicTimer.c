/**************************************************************************************
 * This file is a part of Phone Controlled Car Bare Metal Project for TMP2 *
 **************************************************************************************/
 
#include "periodicTimer.h"

/**
 * @brief Periodic timer on TIM10 initialization.
 * Enable TIM10 clock
 * Set prescaler and auto-reload register
 * Enable update interrupt
 * Enable interrupts
 * Counter enable 
 */
void PERIODIC_TIMER_INIT(void)
{
	RCC->APB2ENR |= (1 << 17);
	
	// 16 Mhz / 1599 + 1 = 10 khz timer clock speed
	TIM10->PSC = 1599;
	
	TIM10->ARR = 999; // todo maybe slow down
	
	TIM10->DIER |= (1 << 0);
	
	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2); // Priority level 2
	// enable TIM10 IRQ from NVIC
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	// Enable Timer 10 module (CEN, bit0)
	TIM10->CR1 |= (1 << 0);
}
