/*
 * timing.c
 *
 * Created: 10.01.2012 21:51:54
 *
 * (c) 2012-2015 by Fabian Huslik
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */  

#include <asf.h>
#include "timing.h"

void RC_in_timing_init(void);

/* Clock Connections

	TIMER_CLOCK1 32 KHz Oscillator
	TIMER_CLOCK2 PBA Clock / 2
	TIMER_CLOCK3 PBA Clock / 8
	TIMER_CLOCK4 PBA Clock / 32
	TIMER_CLOCK5 PBA Clock / 128
	
	PBA is running with sysclk in our case!
	
*/


static volatile uint16_t RC2_A,RC2_B;

 typedef union uSreG_t{
    unsigned long                  sr        ;
    avr32_tc_sr_t                  SR        ;
  }uSreG;


// a test for using the input capture stuff
__attribute__((__interrupt__))
static void isr_RC2(void)
{
	static uint16_t oldA,oldB; // 16 bit timer!!!
	
	uint16_t res;
	
	uSreG sr;
	static uint32_t x;
	
	// this is the cyclic timer interrupt. 
	x = AVR32_TC.channel[TIMERCHANNEL_RC2].sr;	// read only clears the bits 
	
	sr.sr = x;
	
	if(sr.SR.ldras)
	{
		if(sr.SR.mtioa)
		{
			// rising edge
			oldA = AVR32_TC.channel[TIMERCHANNEL_RC2].cv;			
		}
		else
		{
			// falling edge
			res = AVR32_TC.channel[TIMERCHANNEL_RC2].cv;
			res = res - oldA;
				
			if (res < 2*RC_IN_TIMER_COUNT_1MS) // signed, can't be other ;-)  
			{
				RC2_A = res; // just store it, it will be scaled later.
			}
			else
			{
				// leave result as it was...
			}
		}
	}
	
	if(sr.SR.ldrbs)
	{
		if(sr.SR.mtiob)
		{
			// rising edge
			oldB = AVR32_TC.channel[TIMERCHANNEL_RC2].cv;				
		}
		else
		{
			// falling edge
			res = AVR32_TC.channel[TIMERCHANNEL_RC2].cv;
			res = res - oldB;
				
			if (res < 2*RC_IN_TIMER_COUNT_1MS) // signed, can't be other ;-)  
			{
				RC2_B = res; // just store it, it will be scaled later.
			}
			else
			{
				// leave result as it was...
			}		
		}
	}
	
}


void RC_in_timing_init(void)
{
	// Timer 0:
	// Counter for RC measurement
	AVR32_TC.channel[TIMERCHANNEL_RC].CMR.capture.tcclks = AVR32_TC_TCCLKS_TIMER_CLOCK4; // = PBA/32
	AVR32_TC.channel[TIMERCHANNEL_RC].IER.covfs = 0; // enable interrupt 1 = yes
	// AVR32_TC.channel[TIMERCHANNEL_RC].CCR.clken = 1; // enable clock
	AVR32_TC.channel[TIMERCHANNEL_RC].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
	

}


