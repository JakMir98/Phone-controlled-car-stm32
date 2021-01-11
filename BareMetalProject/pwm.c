#include "pwm.h"

#define PWMPERIOD 99

void PWM_INIT(void)
{
	// enable GPIOA clock done in gpio.c
		RCC->APB2ENR |= (1 << 0); // enable TIM1
    GPIOA->MODER |= (2 << PWM_DC_CHANNEL_1_PIN) | (2 << PWM_DC_CHANNEL_2_PIN) | (2 << PWM_LED_FRONT_PIN);  // gpio as alternate function
    
	// Choose Timer1 as Alternative Function for pin 8,9,10
    GPIOA->AFR[1] |= (1 << 0) | (1 << 4) | (1 << 8); 
	
	  TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
		TIM1->CCMR2 &= ~(TIM_CCMR2_CC3S);    //channel 3 is configured as output
	
		/* Channel 1,2& 3 active high */
		TIM1->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
	 
		TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //PWM mode 1
		TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; //PWM mode 1
	 
		TIM1->CR1 = TIM_CR1_CMS_1  | TIM_CR1_ARPE;  //Center-aligned mode 2
		TIM1->CR2 = TIM_CR2_CCPC;                    //CCxE, CCxNE and OCxM bits are preloaded
	 
		TIM1->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE | TIM_BDTR_OSSR;
	 
		TIM1->PSC = 1599;
	 
		TIM1->ARR = PWMPERIOD;  
	 
		TIM1->CCR1 = 0;       // Start PWM duty for channel 1
		TIM1->CCR2 = 0;        // Start PWM duty for channel 2
		TIM1->CCR3 = 0;        // Start PWM duty for channel 3
	 
		/* CH 1-3 Output Compare Enable*/
	 
		TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
		TIM1->CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
		
		TIM1->CR1 = TIM_CR1_CEN;  // Counter enable
}


