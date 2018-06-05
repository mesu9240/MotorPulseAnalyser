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
;  Please read the file named "README.txt", provided in this folder.
;  Please see detailed comments in this file, alongside source code.
;  Application code execution starts in this file.
;
;REVISION HISTORY:
;  $Log: Main.c,v $
;  Revision 1.1.1.1  2005/06/06 09:16:45  VasukiH
;  First release of software
;
;
*/

//Pre-Processor Directives:
//Provide pre-processor directives to include Device header files and
//any application specific header files. Path locations to the Device
//header files are set up within the MPLAB Project>>BuildOptions>>Project
//dialog box. The path should point to X:\\MPLAB C30\support\h\, where
//"X:" is the folder where the MPLAB C30 tools are installed.
#include <p30fxxxx.h>   //"p30fxxxx.h" is a generic header file for dsPIC30F
                        //devices. This file will in turn select the actual
                        //device header file, based on the device selected
                        //by the MPLAB workspace/project. Using the p30fxxxx.h
                        //file makes it easy to migrate a project from one
                        //dsPIC device to another.
#include "system.h"     //"system.h" is a header file defined for this demo
                        //application.
#include "project_types.h"
//Macros for Configuration Fuse Registers:
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers, BOR characteristics etc. The macros are defined within the device
//header files. The configuration fuse registers reside in Flash memory.
_FOSC(CSW_FSCM_OFF & XT_PLL8);  //Run this project using an external crystal
                                //routed via the PLL in 8x multiplier mode
                                //For the 7.3728 MHz crystal we will derive a
                                //throughput of 7.3728e+6*8/4 = 14.74 MIPS(Fcy)
                                //,~67nanoseconds instruction cycle time(Tcy).
_FWDT(WDT_OFF);                 //Turn off the Watch-Dog Timer.
_FBORPOR(MCLR_EN & PWRT_OFF);   //Enable MCLR reset pin and turn off the
                                //power-up timers.
_FGS(CODE_PROT_OFF);            //Disable Code Protection

//Declaration to Link External Functions & Variables:
//Declare functions being used that are defined outside this file, or
//elsewhere in this MPLAB project.
extern SPI_Init(void);
extern UpdateDisplayBuffer(void);
extern WriteUART_to_RS232(void);
extern WriteSPI_to_LCD(void);
extern UART_Init(void);
extern ADC_Init(void);
extern INTx_Init(void);
extern Timer1_Init(void);
extern Timer2_Init(void);
extern init_svpwm_unit(void);
extern set_duty_cycle_u(unsigned int);
extern uart_calibrate_read(int*);
extern uart_calibrate_pre_write(int*);
extern uart_calibrate_write(int*);
extern RET_TYPE calib_chk_ack(void);

//extern char g_u08_S5_sig;
extern char g_u08_uart_received;
extern unsigned int g_u16_uart_received_data[4];
extern int* g_u16_calib_addr;
//Functions and Variables with Global Scope:
//Declare functions in this file that have global scope.
int main (void);
volatile int test_val = 0; // RAM address for testing calibration

int main (void)
{
int i;
        ADPCFG = 0xFFFF;        //After reset all port pins multiplexed
                                //with the A/D converter are configred analog.
                                //We will reconfigure them to be digital
                                //by writing all 1's to the ADPCFG register.
                                //Note: All dsPIC registers are memory mapped.
                                //The address of ADPCFG and other registers
                                //are defined in the device linker script
                                //in your project.

        //Function Delay5ms() available in file, Delay.s
        Delay5ms(200);          //Provide 500ms delay for the LCD to start-up.

        //Function SPI_Init() available in file, SPI_for_LCD.c
        SPI_Init();             //Initialize the SPI module to communicate with
                                //the LCD.

        //Function UART_Init() available in file, UART.c
        UART_Init();            //Initialize the UART module to communicate
                                //with the COM port on the Host PC via an
                                //RS232 cable and the DB9 connector.

        //Function ADC_Init() available in file, A_to_D_Converter.c
        ADC_Init();             //Initialize the A/D converter to convert
                                //signals from the Temperature Sensor and the
                                //Potentiometer.

        //Function INTx_IO_Init() available in file, INTx_IO_pins.c
        INTx_IO_Init();         //Initialize the External interrupt pins and
                                //some I/O pins to accept input from the
                                //switches, S5 and S6 and drive the LEDs, D3
                                //and D4.

        //Function Timer1_Init() & Timer2_Init() available in file, Timers.c
        Timer1_Init();          //Initialize Timer1 to provide "blinking" time
                                //for the LEDs.
        Timer2_Init();          //Initialize Timer2 and Timer3 as a 32-bit
                                //Timer to be used for updating data sent to
                                //the LCD(SPI) and COM(UART) interfaces.

        // svpwm unit
        init_svpwm_unit();
     
        // write test value
	    test_val = 0x6261;
        while (1)
        {
                while (IFS0bits.T3IF == 1)      //Wait until 32-bit Timer
                {                               //interrupt flag bit is set.
                        IFS0bits.T3IF = 0;      //Clear 32-bit timer interrupt
                                                //flag bit
                        T2CONbits.TON = 0;      //Stop 32-bit Timer

                        UpdateDisplayBuffer();  //Write the most recent
                                                //temperature and potentiometer
                                                //values into display buffer

                        //Function WriteUART_to_RS232() available in UART.c
                        //WriteUART_to_RS232();   //Update RS232 via UART
						
						if(g_u08_uart_received == 1)
						{
							g_u08_uart_received = 0; // reset rx flag
							switch (calib_chk_ack())
							{
								case VALID_READ:
									uart_calibrate_read(g_u16_calib_addr);
									break;
								case VALID_WRITE:
									uart_calibrate_pre_write(g_u16_calib_addr);
									break;
								case START_WRITE:
									uart_calibrate_write(g_u16_calib_addr);
									break;
								default:
									U1TXREG = 'F';
								
							}
						}
                        //Function WriteSPI_to_LCD() in file, SPI_for_LCD.c
                        WriteSPI_to_LCD();      //Update the LCD via SPI

                        T2CONbits.TON = 1;      //Start 32-bit Timer again
                }
              
        }

}
