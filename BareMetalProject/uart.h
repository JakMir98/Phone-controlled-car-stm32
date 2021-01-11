#ifndef uart_h
#define uart_h

#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

// uart chars define
#define GO_FORWARD_CHAR 									'u'
#define GO_BACKWARD_CHAR 								'd'
#define TURN_RIGHT_CHAR 											'r'
#define TURN_LEFT_CHAR 												'l'
#define HORN_CHAR 																'h'
#define ADJUST_BRIGHTNESS_CHAR 			'a'
#define LIGHTS_ON_CHAR 												'o'
#define LIGHTS_OFF_CHAR 											'f'
#define DO_NOTHING_CHAR 										'n'
#define RESET_CHAR 															'x'
#define RIGHT_LIGHT_CHAR 											'b'
#define LEFT_LIGHT_CHAR 											'v'

#define USART_TX_PIN 															2
#define USART_RX_PIN 														3

void USART_INIT(void);

void USART2_SendChar(uint8_t);

void USART2_SendString(char *);
	
uint8_t USART2_GetChar(void);

#endif
