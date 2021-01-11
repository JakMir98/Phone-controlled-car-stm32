#ifndef periodicTimer_h
#define periodicTimer_h

#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

void PERIODIC_TIMER_INIT(void);
void TIM1_UP_TIM10_IRQHandler(void);

#endif
