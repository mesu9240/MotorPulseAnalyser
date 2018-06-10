//Pre-Processor Directives:
#include <p30fxxxx.h>
#include "system.h"

void InputCapture_Init(void);
void __attribute__((__interrupt__)) _IC7Interrupt(void);

volatile int gv_cap_evn_len = 0; // time elapsed since last capture event
volatile int gv_cap_evn_val = 0; // history value of the capture event

void InputCapture_Init()
{	
	TRISBbits.TRISB4 = 1; // set RB4 (IC7) port as input
	IC7CON = 0x0;         // reset IC7 channel
	IC7CONbits.ICTMR = 0; // select Timer 3
	IC7CONbits.ICI = 0;   // interrupt on every capture event
	IC7CONbits.ICM = 3;   // capture every rising edge
	IFS1bits.IC7IF = 0;   // clear interrupt flag
	IEC1bits.IC7IE = 1;   // enable interrupt
	
}

void __attribute__((__interrupt__)) _IC7Interrupt(void)
{
	// determine the case of timer overflow
	if (IC7BUF > gv_cap_evn_val)
		gv_cap_evn_len = IC7BUF - gv_cap_evn_val;
	else // timer 3 overflow happened
		gv_cap_evn_len = (IC7BUF + 0xFFFF) - gv_cap_evn_val;
	
	gv_cap_evn_val = IC7BUF;
	
	IFS1bits.IC7IF = 0; // clear interrupt flag for next event
}