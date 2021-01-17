#include "spi.h"

/**
 * @brief SPI initialization,
 * Enable clock for GPIOC, done in gpio.c
 * Set pins directions as output, speed fast and alternate mode on pins PC13, PC14 and PC15
 * Enable SPI2 clock
 * Enable SPI, set master configuration, set software slave management and internal slave select
e 
 */
void SPI_INIT()
{
	// clock for c port on in gpio
	//GPIOB->MODER |= (1<< (2*SPI2_MOSI)) | (1<<(2*SPI2_MISO)) | (1<<(2*SPI2_SCK)); // Set pb13,14,15 to alternate mode (10)
	//GPIOB->OSPEEDR |= (3 << (2*SPI2_MOSI)) | (3<<(2*SPI2_MISO)) | (3 << (2*SPI2_SCK));  // set pb13,14,15 High Speed for PIN PA2 and PA3
	//RCC->APB1ENR |=  (1<< 14); //RCC_APB1ENR_SPI2EN
	
	// set alternate function and fast output
	GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODE15_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->AFR[1] |= (5<< 20) | (5<< 24) | (5<< 28); // alternate function 5 SPI1-4
	
	// turn on spi2 clock
	RCC->APB1ENR |=  RCC_APB1ENR_SPI2EN; 

	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR;
}

/**
 * @brief Control output of MCP23S08 expander
 *
 * @param address to send.
 * @param value to set pin to.
 */
void MCP_Write_Reg(uint8_t addr, uint8_t value)
{
	uint8_t tx_buf[] = {0x40, addr, value};

	GPIOA->ODR &= ~GPIO_ODR_ODR_0;

	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = tx_buf[0];
	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = tx_buf[1];
	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = tx_buf[2];

	GPIOA->ODR |= GPIO_ODR_ODR_0;
}	
