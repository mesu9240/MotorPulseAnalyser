#include <p30fxxxx.h>
#include "system.h"

//Functions and Variables with Global Scope:
void init_svpwm_unit(void);
void set_duty_cycle_u(unsigned int);

//Macros
#define SW_PERIOD 6000

void init_svpwm_unit()
{
   // set pwm timer to up/down mode
   PTCONbits.PTMOD = 2;
   // set pwm timer prescaler to 1:4
   PTCONbits.PTCKPS = 1;
   // set switching period (15 bits)
   PTPERbits.PTPER = SW_PERIOD;
   // configure output pins to be complementary
   PWMCON1bits.PMOD1 = 1;
   PWMCON1bits.PMOD2 = 1;
   PWMCON1bits.PMOD3 = 1;
   // configure output pins to be controlled by PWM
   PWMCON1bits.PEN1L = 1;
   PWMCON1bits.PEN2L = 1;
   PWMCON1bits.PEN3L = 1;
   PWMCON1bits.PEN1H = 1;
   PWMCON1bits.PEN2H = 1;
   PWMCON1bits.PEN3H = 1;
   // start pwm timer
   PTCONbits.PTEN = 1;
}

void set_duty_cycle_u(unsigned int duty_cycle)
{
   if(duty_cycle <= SW_PERIOD)
      PDC1 = duty_cycle;
   else
      PDC1 = SW_PERIOD;
}

void set_pwm_period(unsigned int half_period_ticks)
{
	PTPERbits.PTPER = half_period_ticks;
}