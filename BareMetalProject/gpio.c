#include "gpio.h"

void GPIO_INIT(void)
{
	//PA5 - ON_BOARD_LED_PIN
	//PA7 - LIGHTS_BACK_PIN  
	
	//PB4 - ECHO_PIN
	//PB5 - TRIG_PIN  
	//PB6 - LIGHTS_FRONT_PIN
	
	//PC0 - DC_IN_3_PIN 
	//PC1 - DC_IN_4_PIN 
	//PC2 - DC_IN_1_PIN
	//PC3 - DC_IN_2_PIN 
	//PC7 -  BUZZER_PIN 
	//PC10 - LEFT_LIGHT_PIN 
	//PC11 - RIGHT_LIGHT_PIN 
	//PC13 - USER_BUTTON
	
	// Enable GPIOA, GPIOB, GPIOC clock 
	RCC->AHB1ENR |= (1<<0) | (1<<1) | (1<<2);
	
	//Set the pins as output
	GPIOA->MODER |= (1<<(2*ON_BOARD_LED_PIN)) | (1<<(2*LIGHTS_BACK_PIN));
	GPIOB->MODER |= (1<<(2*TRIG_PIN)) | (1<<(2*LIGHTS_FRONT_PIN));
	GPIOC->MODER |= (1<<(2*DC_IN_1_PIN)) | (1<<(2*DC_IN_2_PIN)) | (1<<(2*DC_IN_3_PIN)) | (1<<(2*DC_IN_4_PIN)) | (1<<(2*BUZZER_PIN)) | (1<<(2*LEFT_LIGHT_PIN)) | (1<<(2*RIGHT_LIGHT_PIN));
	
	// 3. Configure the output mode i.e state, speed, and pull
	GPIOA->OTYPER &= ~((1<<ON_BOARD_LED_PIN) | (1<<LIGHTS_BACK_PIN)); // push pull
	GPIOA->OSPEEDR |= (1<<(2*ON_BOARD_LED_PIN + 1)) | (1<<(2*LIGHTS_BACK_PIN + 1)) ; // fast speed
	GPIOA->PUPDR &= ~((1<<(2*ON_BOARD_LED_PIN)) | (1<<(2*ON_BOARD_LED_PIN+1)) | (1<<(2*LIGHTS_BACK_PIN)) | (1<<(2*LIGHTS_BACK_PIN+1)));
	
	GPIOB->OTYPER &= ~((1<<TRIG_PIN) | (1<<LIGHTS_FRONT_PIN)); // push pull
	GPIOB->OSPEEDR |= (1<<(2*TRIG_PIN + 1)) | (1<<(2*LIGHTS_FRONT_PIN + 1)) ; // fast speed
	GPIOB->PUPDR &= ~((1<<(2*TRIG_PIN)) | (1<<(2*TRIG_PIN+1)) | (1<<(2*LIGHTS_FRONT_PIN)) | (1<<(2*LIGHTS_FRONT_PIN+1)));
	
	GPIOC->OTYPER &= ~((1<<DC_IN_3_PIN) | (1<<DC_IN_4_PIN) | (1<<DC_IN_1_PIN) | (1<<DC_IN_2_PIN) | (1<<BUZZER_PIN) | (1<<LEFT_LIGHT_PIN) | (1<<RIGHT_LIGHT_PIN)); // push pull
	GPIOC->OSPEEDR |= (1<<(2*DC_IN_3_PIN+1)) | (1<<(2*DC_IN_4_PIN+1)) | (1<<(2*DC_IN_1_PIN+1)) | (1<<(2*DC_IN_2_PIN+1)) | (1<<(2*BUZZER_PIN+1)) | (1<<(2*LEFT_LIGHT_PIN+1)) | (1<<(2*RIGHT_LIGHT_PIN+1)) ; // fast speed
	GPIOC->PUPDR &= ~((1<<(2*DC_IN_3_PIN)) | (1<<(2*DC_IN_3_PIN+1)) | (1<<(2*DC_IN_4_PIN)) | (1<<(2*DC_IN_4_PIN+1)) | (1<<(2*DC_IN_1_PIN)) | (1<<(2*DC_IN_1_PIN+1)) | (1<<(2*DC_IN_2_PIN)) | (1<<(2*DC_IN_2_PIN+1)) | (1<<(2*BUZZER_PIN)) | (1<<(2*BUZZER_PIN+1)) | (1<<(2*LEFT_LIGHT_PIN)) | (1<<(2*LEFT_LIGHT_PIN+1)) | (1<<(2*RIGHT_LIGHT_PIN)) | (1<<(2*RIGHT_LIGHT_PIN+1)));
	
	// configure input
	GPIOA->MODER &= ~(1<<(2*ECHO_PIN+1) | (1<<(2*ECHO_PIN))); 
	GPIOA->PUPDR |= (0<<(2*ECHO_PIN));
	
	GPIOC->MODER &= ~(1<<((2*USER_BUTTON)+1) | (1<<(2*USER_BUTTON)) );
	GPIOC->PUPDR |= (0<<(2*USER_BUTTON));
}

