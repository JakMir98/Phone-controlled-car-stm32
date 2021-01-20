/*
 * spi.h
 *
 *  Created on: 19 sty 2021
 *      Author: Admin
 */


#ifndef spi_h_
#define spi_h_


#include "stm32f1xx.h"
#include "system_stm32f1xx.h"

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

void MCP_Write_Reg(uint8_t addr, uint8_t value);
void Andzej_SPI_Init();

#endif /* spi_h_ */
