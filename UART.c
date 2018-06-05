/*
; © 2005  Microchip Technology Inc.
;
; Microchip Technology Inc. (“Microchip”) licenses this software to you
; solely for use with Microchip dsPIC® digital signal controller
; products.  The software is owned by Microchip and is protected under
; applicable copyright laws.  All rights reserved.
;
; SOFTWARE IS PROVIDED “AS IS.”  MICROCHIP EXPRESSLY DISCLAIMS ANY
; WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
; PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
; BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
; DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
; PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
; BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
; ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
;
;
;FILE DESCRIPTION:
; One of the UART modules on the dsPIC30F device is used to refresh
; the Hyperterminal display connected to the RS232 port on the
; dsPICDEM2 development board. Either UART1 or UART2 modules are used.
; When UART1 module is used, this project may optionally use the
; alternate UART tx/rx pins on the dsPIC30F device.
;
;REVISION HISTORY:
;  $Log: UART.c,v $
;  Revision 1.1.1.1  2005/06/06 09:16:45  VasukiH
;  First release of software
;
;
*/

//Pre-Processor Directives:
#include <p30fxxxx.h>
#include "system.h"
#include "project_types.h"

#define BAUDRATE        9600                    //Desired Baud Rate
#define BRGVAL          ((FCY/BAUDRATE)/16)-1   //Formula for U1BRG register
                                                //from dsPIC30F Family
                                                //Reference Manual

//Declaration to Link External Functions & Variables:
extern  unsigned char DisplayData[];

//Functions and Variables with Global Scope:
void UART_Init (void);
void WriteUART_to_RS232(void);
void __attribute__((__interrupt__)) _U1TXInterrupt(void);

unsigned char *UARTCharPtr;
unsigned char g_u08_uart_received = 0;
unsigned int g_u16_uart_received_data[4] = {0,0,0,0};

//Functions

//UART_Init() sets up the UART for a 8-bit data, No Parity, 1 Stop bit
//at 9600 baud with transmitter interrupts enabled
void UART_Init (void)
{
        U1MODE = 0x0000;        //Clear UART2 registers
        U1STA = 0x0000;
		U1STAbits.URXISEL = 3; // gennerate Rx interrupt when RX buffer
							      // is full
        U1MODEbits.ALTIO = 1;   //Enable U1ATX and U1ARX instead of
                                //U1TX and U1RX pins

        U1MODEbits.UARTEN = 1;  //Enable UART1 module
        U1BRG = BRGVAL;         //Load UART1 Baud Rate Generator

        IFS0bits.U1RXIF = 0;    //Clear UART1 Receiver Interrupt Flag
        IFS0bits.U1TXIF = 0;    //Clear UART1 Transmitter Interrupt Flag
        IEC0bits.U1RXIE = 1;    //Disable UART1 Receiver ISR
        IEC0bits.U1TXIE = 1;    //Enable UART1 Transmitter ISR
        U1STAbits.UTXISEL = 1;  //Setup UART1 transmitter to interrupt
                                //when a character is transferred to the
                                //Transmit Shift register and as result,
                                //the transmit buffer becomes empty.

        U1STAbits.UTXEN = 1;    //Enable UART1 transmitter
        UARTCharPtr = &DisplayData[0]; //Initialize UARTCharPtr to point
                                //to the first character in the Display buffer
}

//WriteUART_to_RS232() triggers interrupt-driven UART communication by writing
//the first character in the Display buffer to the UART Transmit register
void WriteUART_to_RS232(void)
{
        if ((UARTCharPtr > &DisplayData[0]) &&
                (UARTCharPtr < &DisplayData[38])) return;
        else
        {
                UARTCharPtr = &DisplayData[0];  //Re-Initialize UART display
                                                //buffer pointer to point to
                                                //the first character
                U1TXREG = *UARTCharPtr++;       //Load the UART transmit
                                                //register with first character
        }
}


//_U1TXInterrupt() is the UART1 Interrupt Service Routine.
//The routine must have global scope in order to be an ISR.
//The ISR name is the same name provided for the module in the device linker
//script.
//The UART1 ISR loads the UART2 4-deep FIFO buffers with the next
//4 characters in the Display buffer unless it encounters a null character.
void __attribute__((__interrupt__)) _U1TXInterrupt(void)
{
        /*int i = 0;
        while ((*UARTCharPtr != '\0') && (i < 4))
        {
                U1TXREG = *UARTCharPtr++;
                i++;
        }*/
        IFS0bits.U1TXIF = 0;    //Clear the UART1 transmitter interrupt flag
}

int* g_u16_calib_addr = 0x0;
int g_s16_calib_write_data = 0;
//global input interface: g_u16_uart_received_data[4], can only be changed by UART RX ISR
//global output interface: g_u16_calib_addr
RET_TYPE calib_chk_ack()
{
	RET_TYPE retCode;
	
	if(g_u16_uart_received_data[0] == 'c')
	{
		switch (g_u16_uart_received_data[1])
		{
		case 'a': // read memory
			retCode = VALID_READ;
			g_u16_calib_addr = (int*)(((g_u16_uart_received_data[2]&0xFF) << 8) | (g_u16_uart_received_data[3]&0xFF));
			break;
		case 'b': // write memory
			retCode = VALID_WRITE;
			g_u16_calib_addr = (int*)(((g_u16_uart_received_data[2]&0xFF) << 8) | (g_u16_uart_received_data[3]&0xFF));
			break;
		case 'c':
			retCode = START_WRITE;
			g_s16_calib_write_data = (int)(((g_u16_uart_received_data[2]&0xFF) << 8) | \
											(g_u16_uart_received_data[3]&0xFF));
			break;
		default:
			retCode = INVALID;
		}
	}
	else
	{
		retCode = INVALID;
	}

	return retCode;
}

volatile int g_s16_CalDataContainer = 0;
char* ptrCalData = (char*)(&g_s16_CalDataContainer);
// void uart_calibrate(void*)
// global interface: unsigned char isBufferAllSent
// send address and content of the data
void uart_calibrate_read(int* ptrData)
{
	ptrCalData = (char*)ptrData;
	// transmit first byte
	g_s16_CalDataContainer = (int)(*ptrCalData);
	U1TXREG = g_s16_CalDataContainer;
	// transmit second byte
	++ptrCalData;
	g_s16_CalDataContainer = (int)(*ptrCalData);
	U1TXREG = g_s16_CalDataContainer;
	U1TXREG = '\r';
	U1TXREG = '\n';
}

//acknowledge write action by sending back the target address
//parameter ptrData: received target address in RAM
void uart_calibrate_pre_write(int* ptrData)
{
	U1TXREG = ((unsigned int)(ptrData) & 0xFF00) >> 8;
	U1TXREG = (unsigned int)(ptrData) & 0xFF;
}

// input interface: g_s16_calib_write_data
void uart_calibrate_write(int* ptrData)
{
	*ptrData = g_s16_calib_write_data;
}

// UART receive interrupt should be triggered by one byte reception
// global output interface: g_u16_uart_received_data
void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
	int i;
	// set reception data signal
	g_u08_uart_received = 1;
	// store the reception data and replete the buffer
	for(i = 0; i < 4; i++)
		g_u16_uart_received_data[i] = U1RXREG;
	// clear interrupt flag
	IFS0bits.U1RXIF = 0;
}