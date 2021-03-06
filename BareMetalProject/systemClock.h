/**************************************************************************************
 * This file is a part of Phone Controlled Car Bare Metal Project for TMP2 *
 **************************************************************************************/

#ifndef systemClock_h
#define systemClock_h

#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

/**************************************************************************************\
* Private definitions
\**************************************************************************************/
#define PLL_M 16
#define PLL_N 336
#define PLL_P 1  // PLLP = 4

/**************************************************************************************\
* Private prototypes
\**************************************************************************************/

/**
 * @brief System clock initialization, setting different clocks speed then 16MHz
 */
void SYS_CLOCK_INIT(void);

#endif
