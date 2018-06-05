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
; This file provides low-leve SPI drivers to interface to the 30F2011
; that controls the text-based 4-bit parallel LCD on the dsPICDEM2
; development board.
;
;REVISION HISTORY:
;  $Log: SPI_for_LCD.c,v $
;  Revision 1.1.1.1  2005/06/06 09:16:45  VasukiH
;  First release of software
;
;
*/

//Pre-Processor Directives:
#include <p30fxxxx.h>
#include "system.h"

#define LCDLINE1CMD     0x80    //Command to set LCD display to Line 1
#define LCDLINE2CMD     0xC0;   //Command to set LCD display to Line 2

//Declaration to Link External Functions & Variables:
extern  Delay5ms (int);
extern  unsigned char DisplayData[];

//Functions and Variables with Global Scope:
void SPI_Init(void);
void WriteSPI_to_LCD(void);

unsigned char *LCDCharPtr;

//SPI_Init Function:
//SPI1 module set up to communicate with the LCD controller on-board.
//Note that when SPI1 is enabled on this device, the UART1 pins will not be
//available due to peripheral multiplexing. So this project utilizes
//UART2 pins, U2TX and U2RX
void SPI_Init(void)
{
        SPI1STAT = 0x0000;
        SPI1CON = 0x0020 ;      //Set the SPI1 module to 8-bit Master mode
        IFS0bits.SPI1IF = 0;    //Clear the SPI1 Interrupt Flag
        IEC0bits.SPI1IE = 0;    //SPI1 ISR processing is not enabled.
                                //SPI1 will be used in polling-mode
        SPI1STATbits.SPIEN = 1; //Turn on the SPI1 module
}

//WriteSPI_to_LCD() Function:
//WriteSPI_to_LCD() writes 32 characters to the 2x16 LCD
//using the SPI1 interface in a polling fashion.
//After each byte is written, the Interrupt Flag bit is polled and the
//next character is written after the interrupt flag bit is set.
void WriteSPI_to_LCD(void)
{
        int temp, i;
        i=0;
        temp = SPI1BUF;
        SPI1STATbits.SPIROV = 0;
        IFS0bits.SPI1IF = 0;
        Delay5us(50);
        SPI1BUF = LCDLINE1CMD;          //First write the command to set cursor
                                        //to Line1 on LCD
        LCDCharPtr = &DisplayData[0];   //Set up LCD character pointer to point
                                        //to the Display buffer
        while(i < 16)
        {
                while (IFS0bits.SPI1IF=0);      //Now write 16 characters
                temp = SPI1BUF;                 //to Line 1 of the LCD
                IFS0bits.SPI1IF = 0;
                SPI1STATbits.SPIROV = 0;
                Delay5ms(1);
                SPI1BUF = *LCDCharPtr++;
                i++;
        }
        while (IFS0bits.SPI1IF==0);
        temp = SPI1BUF;
        IFS0bits.SPI1IF = 0;
        SPI1STATbits.SPIROV = 0;
        temp = *LCDCharPtr++;           //Some characters in the Display buffer
        temp = *LCDCharPtr++;           //are skipped while writing to LCD. CR
        Delay5us(50);                   //and LF are for writing to RS2322 UART
        SPI1BUF = LCDLINE2CMD;          //Next, write the command to set cursor
                                        //to Line2 on LCD
        i = 0;
        while(i < 16)
        {
                while (IFS0bits.SPI1IF==0);     //Now write 16 characters
                temp = SPI1BUF;                 //to Line 2 of the LCD
                IFS0bits.SPI1IF = 0;
                SPI1STATbits.SPIROV = 0;
                Delay5us(50);
                SPI1BUF = *LCDCharPtr++;
                i++;
        }
}


