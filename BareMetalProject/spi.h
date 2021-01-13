/**************************************************************************************
 * This file is a part of Remote Controlled Car Bare Metal Project for TMP2 *
 **************************************************************************************/

#ifndef spi_h
#define spi_h

#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

/**************************************************************************************\
* Private definitions
\**************************************************************************************/
//GPIOB
#define SPI2_MOSI 15
#define SPI2_MISO 14 // no needed  becouse master mode transmit only
#define SPI2_SCK 13

/**************************************************************************************\
* Private prototypes
\**************************************************************************************/

/**
 * @brief SPI2 initialization used 15,14,13 pin on GPIOB.
 */
void SPI_INIT(void);

/**
 * @brief Control output of MCP23S08 expander
 *
 * @param address to send.
 * @param value to set pin to.
 */
void MCP_Write_Reg(uint8_t, uint8_t);

#endif
