#ifndef spi_h
#define spi_h

#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

//GPIOB
#define SPI2_MOSI 15
#define SPI2_MISO 14 // no needed  becouse master mode transmit only
#define SPI2_SCK 13

void SPI_INIT(void);

void MCP_Write_Reg(uint8_t, uint8_t);

#endif
