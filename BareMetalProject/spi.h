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

//code for SPI handling
#define MCP_IODIR 		0x00
#define MCP_IPOL 		0x01
#define MCP_GPINTEN		0x02
#define MCP_DEFVAL 		0x03
#define MCP_INTCON 		0x04
#define MCP_IOCON 		0x05
#define MCP_GPPU 		0x06
#define MCP_INTF 		0x07
#define MCP_INTCAP 		0x08
#define MCP_GPIO 		0x09
#define MCP_OLAT 		0x0A
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
