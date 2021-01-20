/*
 * spi.c
 *
 *  Created on: 19 sty 2021
 *      Author: Admin
 */
#include "spi.h"

//SPI
void Andzej_SPI_Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;

	GPIOA->CRL |= GPIO_CRL_MODE0_0 | GPIO_CRL_MODE5_0 | GPIO_CRL_MODE7_0;
	GPIOA->CRL &= ~(GPIO_CRL_CNF0_0 | GPIO_CRL_CNF5_0 | GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0);
	GPIOA->CRL |= GPIO_CRL_CNF5_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;

	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR;
}

void MCP_Write_Reg(uint8_t addr, uint8_t value)
{
	uint8_t tx_buf[] = {0x40, addr, value};

	GPIOA->ODR &= ~GPIO_ODR_ODR0;

	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = tx_buf[0];
	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = tx_buf[1];
	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = tx_buf[2];

	GPIOA->ODR |= GPIO_ODR_ODR0;
}

