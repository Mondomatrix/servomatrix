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
 
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "spi.h"
#include "mcp23s17.h"
#include "auart.h"

#ifndef cbi
#define cbi(register,bit)	register &= ~(_BV(bit))
#endif
#ifndef sbi
#define sbi(register,bit)	register |= (_BV(bit))
#endif

#define GroupMask 0x00
#define MaxPrepulse 60
#define PrePulse1 0
#define Pulse1 1
#define PrePulse2 2
#define Pulse2 3
#define PrePulse3 4
#define Pulse3 5
#define PrePulse4 6
#define Pulse4 7
#define PulseWait1 8
#define PulseWait2 9
#define PulseWait3 10
#define PulseWait4 11

#define PULSEOCRA 0x09
#define NORMOCRA 0x13

#define SERIAL_STATE_WAITING 0
#define SERIAL_STATE_COMMAND 1
#define SERIAL_STATE_DATA 2

volatile uint8_t MyAddress = 0;
volatile uint8_t CommandByte = 0;
volatile uint8_t DataBuffer[256];
volatile uint8_t DataCount = 0;
volatile uint8_t BytesReceived = 0;
volatile uint8_t ProcessCommand = 0;

volatile uint8_t ServoVal[64];

volatile uint8_t RoundCount = 0;
volatile uint8_t Group = 0;
volatile uint8_t PulseState = 0;
volatile uint8_t DoIO = 0;
volatile uint8_t IOPulseState = 0;

void IOSetup(void);
void TimerSetup(void);
uint8_t SerialLengthFromCommand(uint8_t CmdByte);
void SerialStateMachine(uint8_t SerialByte, uint8_t NinthBit);
uint8_t GetMyAddress(void);

int main(void)
{
	uint8_t DataBufferA;
	uint8_t DataBufferB;
	uint8_t i, j, jdot, kdot;

	IOSetup();
	SPISetup();
	AUARTSetup();

	MCP23S17SetupAll();
	MCP23S17Setup(0);
	MCP23S17Setup(1);
	MCP23S17Setup(2);
	MCP23S17Setup(3);

	MyAddress = GetMyAddress();

	for(i=0;i<128;i++)
	{
		DataBuffer[i] = 0;
		DataBuffer[i+128] = 0;
	}

	for(i=0;i<64;i++)
	{
		ServoVal[i] = 127;
	}

	TimerSetup();

/*	ServoVal[0] = 0;
	ServoVal[15] = 255;
	ServoVal[17] = 0;
	ServoVal[18] = 255;
	ServoVal[34] = 0;
	ServoVal[35] = 255;
	ServoVal[50] = 0;
	ServoVal[51] = 255;
*/
	while(1)
	{
		if(ProcessCommand == 1)
		{
			ProcessCommand = 0;
			//based on command byte, execute the function
			if(CommandByte == 1)
			{
				for(j=0;j<4;j++)
				{
					jdot = j * 16;
					ServoVal[0+jdot] = DataBuffer[7+jdot];
					ServoVal[1+jdot] = DataBuffer[6+jdot];
					ServoVal[2+jdot] = DataBuffer[5+jdot];
					ServoVal[3+jdot] = DataBuffer[4+jdot];
					ServoVal[4+jdot] = DataBuffer[3+jdot];
					ServoVal[5+jdot] = DataBuffer[2+jdot];
					ServoVal[6+jdot] = DataBuffer[1+jdot];
					ServoVal[7+jdot] = DataBuffer[0+jdot];
					ServoVal[8+jdot] = DataBuffer[8+jdot];
					ServoVal[9+jdot] = DataBuffer[9+jdot];
					ServoVal[10+jdot] = DataBuffer[10+jdot];
					ServoVal[11+jdot] = DataBuffer[11+jdot];
					ServoVal[12+jdot] = DataBuffer[12+jdot];
					ServoVal[13+jdot] = DataBuffer[13+jdot];
					ServoVal[14+jdot] = DataBuffer[14+jdot];
					ServoVal[15+jdot] = DataBuffer[15+jdot];
				}
			} else if(CommandByte == 2) {
				jdot = DataBuffer[0];
				kdot = jdot % 16;
				jdot = jdot - kdot;
				if(kdot >= 8)
				{
					ServoVal[jdot+kdot] = DataBuffer[1];
				} else {
					ServoVal[jdot+(7-kdot)] = DataBuffer[1];
				}
			}
			CommandByte = 0;
			BytesReceived = 0;
			DataCount = 0;
		}

		if(DoIO == 1)
		{
			DoIO = 0;
			switch(IOPulseState)
			{
			case PrePulse1:
					while((SPSR & (1<<SPIF))==0);
					SPDR = 0;
					while((SPSR & (1<<SPIF))==0);
					SPDR = 0;
					while((SPSR & (1<<SPIF))==0);
 					SPISSOn();
 					_delay_us(10);
					MCP23S17WriteIO(0, 255, 255);
					break;
			case Pulse1:
					if(RoundCount < 250)
					{
						DataBufferA = 0;
						DataBufferB = 0;

						if(RoundCount < ServoVal[0]) sbi(DataBufferA, 0);
						if(RoundCount < ServoVal[1]) sbi(DataBufferA, 1);
						if(RoundCount < ServoVal[2]) sbi(DataBufferA, 2);
						if(RoundCount < ServoVal[3]) sbi(DataBufferA, 3);
						if(RoundCount < ServoVal[4]) sbi(DataBufferA, 4);
						if(RoundCount < ServoVal[5]) sbi(DataBufferA, 5);
						if(RoundCount < ServoVal[6]) sbi(DataBufferA, 6);
						if(RoundCount < ServoVal[7]) sbi(DataBufferA, 7);

						while((SPSR & (1<<SPIF))==0);
						SPDR = DataBufferA;

						if(RoundCount < ServoVal[8]) sbi(DataBufferB, 0);
						if(RoundCount < ServoVal[9]) sbi(DataBufferB, 1);
						if(RoundCount < ServoVal[10]) sbi(DataBufferB, 2);
						if(RoundCount < ServoVal[11]) sbi(DataBufferB, 3);
						if(RoundCount < ServoVal[12]) sbi(DataBufferB, 4);
						if(RoundCount < ServoVal[13]) sbi(DataBufferB, 5);
						if(RoundCount < ServoVal[14]) sbi(DataBufferB, 6);
						if(RoundCount < ServoVal[15]) sbi(DataBufferB, 7);

						while((SPSR & (1<<SPIF))==0);
						SPDR = DataBufferB;
					} else {
						while((SPSR & (1<<SPIF))==0);
						SPDR = 0;
						while((SPSR & (1<<SPIF))==0);
						SPDR = 0;
					}
											
					break;
			case PrePulse2:
					while((SPSR & (1<<SPIF))==0);
					SPISSOn();
					MCP23S17WriteIO(1, 255, 255);
					break;
			case Pulse2:
					if(RoundCount < 250)
					{
						DataBufferA = 0;
						DataBufferB = 0;
						if(RoundCount < ServoVal[16]) sbi(DataBufferA, 0);
						if(RoundCount < ServoVal[17]) sbi(DataBufferA, 1);
						if(RoundCount < ServoVal[18]) sbi(DataBufferA, 2);
						if(RoundCount < ServoVal[19]) sbi(DataBufferA, 3);
						if(RoundCount < ServoVal[20]) sbi(DataBufferA, 4);
						if(RoundCount < ServoVal[21]) sbi(DataBufferA, 5);
						if(RoundCount < ServoVal[22]) sbi(DataBufferA, 6);
						if(RoundCount < ServoVal[23]) sbi(DataBufferA, 7);
	
						while((SPSR & (1<<SPIF))==0);
						SPDR = DataBufferA;
	
						if(RoundCount < ServoVal[24]) sbi(DataBufferB, 0);
						if(RoundCount < ServoVal[25]) sbi(DataBufferB, 1);
						if(RoundCount < ServoVal[26]) sbi(DataBufferB, 2);
						if(RoundCount < ServoVal[27]) sbi(DataBufferB, 3);
						if(RoundCount < ServoVal[28]) sbi(DataBufferB, 4);
						if(RoundCount < ServoVal[29]) sbi(DataBufferB, 5);
						if(RoundCount < ServoVal[30]) sbi(DataBufferB, 6);
						if(RoundCount < ServoVal[31]) sbi(DataBufferB, 7);
	
						while((SPSR & (1<<SPIF))==0);
						SPDR = DataBufferB;
					} else {
						while((SPSR & (1<<SPIF))==0);
						SPDR = 0;
						while((SPSR & (1<<SPIF))==0);
						SPDR = 0;
					}

					break;
			case PrePulse3:
					while((SPSR & (1<<SPIF))==0);
					SPISSOn();
					MCP23S17WriteIO(2, 255, 255);
					break;
			case Pulse3:
					if(RoundCount < 250)
					{
						DataBufferA = 0;
						DataBufferB = 0;
	
						if(RoundCount < ServoVal[32]) sbi(DataBufferA, 0);
						if(RoundCount < ServoVal[33]) sbi(DataBufferA, 1);
						if(RoundCount < ServoVal[34]) sbi(DataBufferA, 2);
						if(RoundCount < ServoVal[35]) sbi(DataBufferA, 3);
						if(RoundCount < ServoVal[36]) sbi(DataBufferA, 4);
						if(RoundCount < ServoVal[37]) sbi(DataBufferA, 5);
						if(RoundCount < ServoVal[38]) sbi(DataBufferA, 6);
						if(RoundCount < ServoVal[39]) sbi(DataBufferA, 7);
	
						while((SPSR & (1<<SPIF))==0);
						SPDR = DataBufferA;
	
						if(RoundCount < ServoVal[40]) sbi(DataBufferB, 0);
						if(RoundCount < ServoVal[41]) sbi(DataBufferB, 1);
						if(RoundCount < ServoVal[42]) sbi(DataBufferB, 2);
						if(RoundCount < ServoVal[43]) sbi(DataBufferB, 3);
						if(RoundCount < ServoVal[44]) sbi(DataBufferB, 4);
						if(RoundCount < ServoVal[45]) sbi(DataBufferB, 5);
						if(RoundCount < ServoVal[46]) sbi(DataBufferB, 6);
						if(RoundCount < ServoVal[47]) sbi(DataBufferB, 7);
	
						while((SPSR & (1<<SPIF))==0);
						SPDR = DataBufferB;
					} else {
						while((SPSR & (1<<SPIF))==0);
						SPDR = 0;
						while((SPSR & (1<<SPIF))==0);
						SPDR = 0;
					}

					break;
			case PrePulse4:
					while((SPSR & (1<<SPIF))==0);
					SPISSOn();
					MCP23S17WriteIO(3, 255, 255);
					break;
			case Pulse4:
					if(RoundCount < 250)
					{
						DataBufferA = 0;
						DataBufferB = 0;
	
						if(RoundCount < ServoVal[48]) sbi(DataBufferA, 0);
						if(RoundCount < ServoVal[49]) sbi(DataBufferA, 1);
						if(RoundCount < ServoVal[50]) sbi(DataBufferA, 2);
						if(RoundCount < ServoVal[51]) sbi(DataBufferA, 3);
						if(RoundCount < ServoVal[52]) sbi(DataBufferA, 4);
						if(RoundCount < ServoVal[53]) sbi(DataBufferA, 5);
						if(RoundCount < ServoVal[54]) sbi(DataBufferA, 6);
						if(RoundCount < ServoVal[55]) sbi(DataBufferA, 7);
	
						while((SPSR & (1<<SPIF))==0);
						SPDR = DataBufferA;
	
						if(RoundCount < ServoVal[56]) sbi(DataBufferB, 0);
						if(RoundCount < ServoVal[57]) sbi(DataBufferB, 1);
						if(RoundCount < ServoVal[58]) sbi(DataBufferB, 2);
						if(RoundCount < ServoVal[59]) sbi(DataBufferB, 3);
						if(RoundCount < ServoVal[60]) sbi(DataBufferB, 4);
						if(RoundCount < ServoVal[61]) sbi(DataBufferB, 5);
						if(RoundCount < ServoVal[62]) sbi(DataBufferB, 6);
						if(RoundCount < ServoVal[63]) sbi(DataBufferB, 7);
	
						while((SPSR & (1<<SPIF))==0);
						SPDR = DataBufferB;
					} else {
						while((SPSR & (1<<SPIF))==0);
						SPDR = 0;
						while((SPSR & (1<<SPIF))==0);
						SPDR = 0;
					}

					break;
			case PulseWait1:
					while((SPSR & (1<<SPIF))==0);
					SPISSOn();
					break;					
			default:
				break;
			}
		}
	}
	return 0;
}

uint8_t GetMyAddress(void)
{
	uint8_t addressBuffer = 0;
	//read the address, return it
	
	addressBuffer = PINC & 0x3F;
	addressBuffer |= ((PIND & 0x18) << 3);
	
	return addressBuffer;
}

void SerialStateMachine(uint8_t SerialByte, uint8_t NinthBit)
{
	static uint8_t SerialState = SERIAL_STATE_WAITING;
	
	switch(SerialState)
	{
		case SERIAL_STATE_WAITING:
			//make sure MPCM is set
			//check for address match
			if(SerialByte == MyAddress)
			{
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (0 << MPCM0));
				SerialState = SERIAL_STATE_COMMAND;
			} else {
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
				SerialState = SERIAL_STATE_WAITING;
			}
			//no match, resume
			//if match, clear MPCM, state = STATE_ADDRESS
			break;
		case SERIAL_STATE_COMMAND:
			//check ninth bit. if clear, continue, else, state = SERIAL_STATE_WAITING, set MPCM
			if(NinthBit != 0)
			{
				SerialState = SERIAL_STATE_WAITING;
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
			} else {
			//commandbyte = data
				CommandByte = SerialByte;
			//state = SERIAL_STATE_DATA
				SerialState = SERIAL_STATE_DATA;
			//set DataCount based on command
				DataCount = SerialLengthFromCommand(SerialByte);
				BytesReceived = 0;
			}
			break;
		case SERIAL_STATE_DATA:
			//check ninth bit. if clear, continue, else, state = SERIAL_STATE_WAITING, DataCount = 0
			if(NinthBit == 0)
			{
				DataBuffer[BytesReceived] = SerialByte;
			//DataBuffer[BytesReceived] = byte
				BytesReceived++;
				if(DataCount == BytesReceived)
				{
					ProcessCommand = 1;
					SerialState = SERIAL_STATE_WAITING;
					UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
				} else {
					SerialState = SERIAL_STATE_DATA;
				}
			} else {
				SerialState = SERIAL_STATE_WAITING;
				UCSR0A = ((0 << TXC0) | (1 << U2X0) | (1 << MPCM0));
			}
			//bytesreceived++
			//if datacount == bytesrecieved then call command processor, bytesreceived = 0, datacount = 0, state = SERIAL_STATE_WAITING, set MPCM
			//else state = SERIAL_STATE_DATA
			break;
		default:
			SerialState = SERIAL_STATE_WAITING; // also set MPCM
			break;
	}
}

uint8_t SerialLengthFromCommand(uint8_t CmdByte)
{
	//return proper data length for a command
	if(CmdByte == 1)
	{
		return 64;
	} else if(CmdByte == 2) {
		return 2;
	} else {
		return 0;
	}
}

void IOSetup(void)
{
	DDRC = 0;
	cbi(DDRD, 4);
}

void TimerSetup(void)
{
	TCCR0A = ((0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (0 << WGM00));
	TCCR0B = ((0 << FOC0A) | (0 << FOC0B) | (0 << WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00));
	TIMSK0 = ((0 << OCIE0B) | (1 << OCIE0A) | (0 << TOIE0));
	OCR0A = NORMOCRA;
	sei();
}

ISR(USART_RX_vect)
{
	uint8_t IntDataByte, IntNinthBit;
	IntDataByte = UDR0;
	IntNinthBit = UCSR0A & 0x01;
	SerialStateMachine(IntDataByte, IntNinthBit);
}

ISR(TIMER0_COMPA_vect)
{
	

	switch(PulseState)
	{
	case PrePulse1:
			if(RoundCount == 0)
			{
				DoIO = 1; IOPulseState = PulseState;
			} else if(RoundCount >= MaxPrepulse) {
				PulseState = Pulse1;
				RoundCount = 0;
			}
			break;
	case Pulse1:
			DoIO = 1; IOPulseState = PulseState;

			if(RoundCount==254)
			{
				PulseState = PrePulse2;
			} else {
				DoIO = 1; IOPulseState = PulseState;
			}			

			break;
	case PrePulse2:
			if(RoundCount == 255)
			{
				DoIO = 1; IOPulseState = PulseState;
			} else if(RoundCount >= MaxPrepulse) {
				PulseState = Pulse2;
				RoundCount = 0;
			}
			break;
	case Pulse2:
			DoIO = 1; IOPulseState = PulseState;

			if(RoundCount==254)
			{
				PulseState = PrePulse3;
			}
			
			break;
	case PrePulse3:
			if(RoundCount == 255)
			{
				DoIO = 1; IOPulseState = PulseState;
			} else if(RoundCount >= MaxPrepulse) {
				PulseState = Pulse3;
				RoundCount = 0;
			}
			break;
	case Pulse3:
			DoIO = 1; IOPulseState = PulseState;

			if(RoundCount==254)
			{
				PulseState = PrePulse4;
			}

			break;
	case PrePulse4:
			if(RoundCount == 255)
			{
				DoIO = 1; IOPulseState = PulseState;
			} else if(RoundCount >= MaxPrepulse) {
				PulseState = Pulse4;
				RoundCount = 0;
			}
			break;
	case Pulse4:
			DoIO = 1; IOPulseState = PulseState;

			if(RoundCount==254)
			{
				PulseState = PulseWait1;
			}

			break;
	case PulseWait1:
			if(RoundCount == 255)
			{
				DoIO = 1;
				IOPulseState = PulseState;
			}
			if(RoundCount == 255)
			{
				PulseState = PulseWait2;
			}
			break;
	case PulseWait2:
			if(RoundCount == 255)
			{
				PulseState = PulseWait3;
			}
			break;
	case PulseWait3:
			if(RoundCount == 255)
			{
				PulseState = PulseWait4;
			}
			break;
	case PulseWait4:
			if(RoundCount == 255)
			{
				PulseState = PrePulse1;
			}
			break;
	default:
		break;
	}
	RoundCount++;
}
