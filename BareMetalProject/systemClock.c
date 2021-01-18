/**************************************************************************************
 * This file is a part of Phone Controlled Car Bare Metal Project for TMP2 *
 **************************************************************************************/
 
#include "systemClock.h"

void SYS_CLOCK_INIT(void)
{
SystemInit();
	 /* Enable HSI (CR: bit 16) */
    RCC->CR |= (1 << 0);
    /* Wait till HSI is ready (CR: bit 17) */
    while(!(RCC->CR & (1 << 1)));

    /* set voltage scale to 1 for max frequency */
    /* first enable power interface clock (APB1ENR:bit 28) */
    RCC->APB1ENR |= (1 << 28);

    /* then set voltage scale to 1 for max frequency (PWR_CR:bit 14)
     * 00: Reserved (Scale 3 mode selected)
				01: Scale 3 mode <= 64 MHz
				10: Scale 2 mode (reset value) <= 84 MHz
				11: Scale 1 mode <= 100 MHz
     */
    PWR->CR |= (1 << 15) ;
	  PWR->CR &= ~(1 << 14) ;

    /* Configure Flash
     * prefetch enable (ACR:bit 8)
     * instruction cache enable (ACR:bit 9)
     * data cache enable (ACR:bit 10)
     * set latency to 2 wait states (ARC:bits 2:0)
     */
    FLASH->ACR = (1 << 8) | (1 << 9) | (1 << 10 ) | (2 << 0);
		
		 /* set AHB prescaler to /1 (CFGR:bits 7:4) */
    RCC->CFGR |= (0 << 4);
    /* set ABP low speed prescaler to /2 (APB1) (CFGR:bits 12:10) */
    RCC->CFGR |= (4 << 10);
    /* set ABP high speed prescaper to /1 (ABP2) (CFGR:bits 15:13) */
    RCC->CFGR |= (0 << 13);
		
		RCC->PLLCFGR = (PLL_M <<0) | (PLL_N << 6) | (PLL_P <<16) | (1<<22);
		
		RCC->CR |= (1<<24);
		while (!(RCC->CR & (1<<25)));

    /* Select the main PLL as system clock source, (CFGR:bits 1:0)
     * 0b00 - HSI
     * 0b01 - HSE
     * 0b10 - PLL
     */
    RCC->CFGR |=  (2  << 0);
    /* Wait till the main PLL is used as system clock source (CFGR:bits 3:2) */
    while (!(RCC->CFGR & (2 << 2)));
		SystemCoreClock = 84000000;
		SystemCoreClockUpdate();
}

void test()
{
	SystemInit();
	 /* Enable HSE (CR: bit 16) */
    RCC->CR |= (1 << 16);
    /* Wait till HSE is ready (CR: bit 17) */
    while(!(RCC->CR & (1 << 17)));

    /* set voltage scale to 1 for max frequency */
    /* first enable power interface clock (APB1ENR:bit 28) */
    RCC->APB1ENR |= (1 << 28);

    /* then set voltage scale to 1 for max frequency (PWR_CR:bit 14)
     * 00: Reserved (Scale 3 mode selected)
				01: Scale 3 mode <= 64 MHz
				10: Scale 2 mode (reset value) <= 84 MHz
				11: Scale 1 mode <= 100 MHz
     */
    PWR->CR |= (1 << 15) ;
	  PWR->CR &= ~(1 << 14) ;

    /* Configure Flash
     * prefetch enable (ACR:bit 8)
     * instruction cache enable (ACR:bit 9)
     * data cache enable (ACR:bit 10)
     * set latency to 2 wait states (ARC:bits 2:0)
     *   see Table 10 on page 80 in RM0090
     */
    FLASH->ACR = (1 << 8) | (1 << 9) | (1 << 10 ) | (2 << 0);
		
		 /* set AHB prescaler to /1 (CFGR:bits 7:4) */
    RCC->CFGR |= (0 << 4);
    /* set ABP low speed prescaler to /2 (APB1) (CFGR:bits 12:10) */
    RCC->CFGR |= (4 << 10);
    /* set ABP high speed prescaper to /1 (ABP2) (CFGR:bits 15:13) */
    RCC->CFGR |= (0 << 13);
		
		RCC->PLLCFGR = (PLL_M <<0) | (PLL_N << 6) | (PLL_P <<16) | (1<<22);
		
		RCC->CR |= (1<<24);
		while (!(RCC->CR & (1<<25)));

    /* Select the main PLL as system clock source, (CFGR:bits 1:0)
     * 0b00 - HSI
     * 0b01 - HSE
     * 0b10 - PLL
     */
    RCC->CFGR |=  (2  << 0);
    /* Wait till the main PLL is used as system clock source (CFGR:bits 3:2) */
    while (!(RCC->CFGR & (2 << 2)));
		SystemCoreClock = 84000000;
		SystemCoreClockUpdate();
}
