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
; This is a header file defined for this project. It contains
; definitions applicable to multiple source files in this project.
;
;REVISION HISTORY:
;  $Log: System.h,v $
;  Revision 1.1.1.1  2005/06/06 09:16:45  VasukiH
;  First release of software
;
;
*/
#include <p30fxxxx.h>

//Defines for System Clock Timing -
//For oscillator configuration XT x PLL8 mode,
//Device Throughput in MIPS = Fcy = 7372800*8/4 = ~14.74 MIPS
//Instruction Cycle time = Tcy = 1/(Fcy) = ~68 nanoseconds
#define XTFREQ          7372800         //On-board Crystal frequency
#define PLLMODE         8               //On-chip PLL setting
#define FCY             XTFREQ*PLLMODE/4        //Instruction Cycle Frequency

//Defines that equate Switches on board to specific interrupt pins on device
#define Switch_S5       PORTEbits.RE8   //Switch S5 is connected to RE8
#define Switch_S6       PORTDbits.RD0   //Switch S6 is connected to RD0






