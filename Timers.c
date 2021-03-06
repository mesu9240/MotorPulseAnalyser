/*
; � 2005  Microchip Technology Inc.
;
; Microchip Technology Inc. (�Microchip�) licenses this software to you
; solely for use with Microchip dsPIC� digital signal controller
; products.  The software is owned by Microchip and is protected under
; applicable copyright laws.  All rights reserved.
;
; SOFTWARE IS PROVIDED �AS IS.�  MICROCHIP EXPRESSLY DISCLAIMS ANY
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
; This file contains General-purpose timer set up and interrupt
; service routines for 16-bit Timer 1. Timer 1 interrupts are
; used to toggle both the LEDs on the dsPICDEM2 board.
; Timer2 and Timer3 are used together in 32-bit mode with interrupts
; disabled. The 32-bit Timer is used to decide how often the LCD and
; RS232 displays need to be refreshed.
;
;REVISION HISTORY:
;  $Log: Timers.c,v $
;  Revision 1.1.1.1  2005/06/06 09:16:45  VasukiH
;  First release of software
;
;
*/

//Pre-Processor Directives:
#include <p30fxxxx.h>
#include "system.h"

//Functions and Variables with Global Scope:
void Timer1_Init(void);
void Timer2_Init(void);
void __attribute__((__interrupt__)) _T1Interrupt(void);

//Timer1_Init() sets up Timer1 to count up to the maximum 16-bit value, 0xFFFF,
//and interrupt the CPU. ISR processing is enabled for Timer 1.
void Timer1_Init(void)
{
        T1CON = 0x0020;         //Timer1 set up to count on instruction cycle
                                //edge with 1:64 prescaler applied initially.
        PR1 = 0xFFFF;           //Period Register, PR1, set to maximum count
        IFS0bits.T1IF = 0;      //Clear the Timer1 Interrupt Flag
        IEC0bits.T1IE = 1;      //Enable Timer1 Interrupt Service Routine
        T1CONbits.TON=1;        //Start Timer 1
}

//Timer2_Init() sets up Timer2 and Timer3 to count up to a 32-bit value,
//0x003FFFFF with a prescaler of 1:1. 32-bit Timer ISR is disabled.
void Timer2_Init(void)
{
        T2CON = 0x0000;         //32-bit Timer3:Timer2 pair set up
        T2CONbits.T32 = 1;      //to increment every instruction cycle
        PR3 = 0x003F;
        PR2 = 0xFFFF;           //Period Register, PR3:PR2, set to 0x003FFFFF
        IFS0bits.T3IF = 0;      //Clear the Timer3 Interrupt Flag
        IEC0bits.T3IE = 0;      //Disable Timer3 Interrup Service Routine
        T2CONbits.TON=1;        //Start 32-bit timer, setting Timer 2 ON bit
}

//_T1Interrupt() is the Timer1 Interrupt Service Routine
//The routine must have global scope in order to be an ISR.
//The ISR name is the same name provided for the module in the device linker
//script.
//At every Timer1 interrupt event, the state of the two LEDs, D3 and D4,
//is toggled/flipped.
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
        LATBbits.LATB0 =~ LATBbits.LATB0;       //Toggle LATB0 (LED D3)
        LATBbits.LATB1 =~ LATBbits.LATB1;       //Toggle LATB1 (LED D4)
        IFS0bits.T1IF = 0;                      //Clear Timer1 Interrupt Flag
}


