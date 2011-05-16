/*
 This file is part of the LEDMatrix firmware.
 
 The LEDMatrix firmware is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 The LEDMatrix firmware is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with the LEDMatrix firmware.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef RBGMCP23S17
#define RBGMCP23S17

#include <stdint.h>
#include "spi.h"

#define MCP23S17_IODIRA		0x00
#define MCP23S17_IODIRB		0x01
#define MCP23S17_IPOLA		0x02
#define MCP23S17_IPOLB		0x03
#define MCP23S17_GPINTENA	0x04
#define MCP23S17_GPINTENB	0x05
#define MCP23S17_DEFVALA	0x06
#define MCP23S17_DEFVALB	0x07
#define MCP23S17_INTCONA	0x08
#define MCP23S17_INTCONB	0x09
#define MCP23S17_IOCON		0x0A
#define MCP23S17_GPPUA		0x0C
#define MCP23S17_GPPUB		0x0D
#define MCP23S17_INTFA		0x0E
#define MCP23S17_INTFB		0x0F
#define MCP23S17_INTCAPA	0x10
#define MCP23S17_INTCAPB	0x11
#define MCP23S17_GPIOA		0x12
#define MCP23S17_GPIOB		0x13
#define MCP23S17_OLATA		0x14
#define MCP23S17_OLATB		0x15

void MCP23S17SetupAll(void);
void MCP23S17Setup(uint8_t address);
void MCP23S17WriteRegister (uint8_t address, uint8_t reg, uint8_t data);
uint8_t MCP23S17ReadRegister(uint8_t address, uint8_t reg);
void MCP23S17WriteIO (uint8_t address, uint8_t Aport, uint8_t Bport);
void MCP23S17QuickWriteIO (uint8_t Aport, uint8_t Bport);

#endif
