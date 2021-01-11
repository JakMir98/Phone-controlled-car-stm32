#ifndef systemClock_h
#define systemClock_h

#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

#define PLL_M 	16
#define PLL_N 	336
#define PLL_P 	1  // PLLP = 4

void SYS_CLOCK_INIT(void);

#endif
