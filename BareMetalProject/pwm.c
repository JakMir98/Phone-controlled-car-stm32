#include "pwm.h"

#define PWMPERIOD 100
#define SAMPLE 100

void PWM_INIT(void)
{
	// enable GPIOA clock
	 //RCC->AHB1ENR |= (1 << 3); already done in gpio
    GPIOA->MODER |= (2 << PWM_DC_CHANNEL_1_PIN) | (2 << PWM_DC_CHANNEL_2_PIN) | (2 << PWM_LED_FRONT_PIN);  
    
	// Choose Timer1 as Alternative Function for pin 8,9,10
    GPIOA->AFR[1] |= (1 << 0) | (1 << 4) | (1 << 8);
    // enable TIM1 clock (bit0)
    RCC->APB2ENR |= (1 << 0);

    // fCK_PSC / (PSC[15:0] + 1)
    // 84 Mhz / 8399 + 1 = 10 khz timer clock speed
    TIM1->PSC = 1599;
    // set period
    TIM1->ARR = PWMPERIOD;
    // set duty cycle on channel 1
    TIM1->CCR1 = 1;
		TIM1->CCR2 = 1;
		TIM1->CCR3 = 1;

    // enable channel 1 in capture/compare register
    // set oc1 oc2 oc3 mode as pwm (0b110 or 0x6 in bits 6-4)
    TIM1->CCMR1 |= (6 << 4) | (6 << 12);
		TIM1->CCMR2 |= (6 << 4);
    // enable oc1 preload bit 3
    TIM1->CCMR1 |= (1 << 3) | (1<< 11);
		TIM1->CCMR2 |= (1 << 3) ;
    // enable capture/compare ch1 output
    TIM1->CCER |= (1 << 0)  | (1 << 4) | (1 << 8);
    // enable update interrupt
    //TIM1->DIER |= (1 << 0);

    //NVIC_SetPriority(TIM, 2); // Priority level 2
    // enable TIM1 IRQ from NVIC
    //NVIC_EnableIRQ(TIM1_IRQn);

    // Enable Timer 1 module (CEN, bit0)
    TIM1->CR1 |= (1 << 0);
}
