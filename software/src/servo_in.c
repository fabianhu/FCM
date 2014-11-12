/*
 * servo_in.c
 *
 * Created: 08.01.2012 21:31:24
 *
 * (c) 2012-2014 by Fabian Huslik
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
#include "servo_in.h"
#include "modules/SUMD.h"
#include "modules/SPEKTRUM.h"
#include "modules/eic.h"
#include "timing.h"
#include "UsartRC.h"
#include "TaskControl.h"
#include "config.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/emergency.h"


#define SERVO_1ms 0L
#define SERVO_2ms 10000L
#define SERVO_CENTER SERVO_2ms/2

int16_t servo_in_channel_raw[12]; // for sum signals up to 12 channels. in raw scaling!

RXType_t RXType = RXType_Unknown;
bool RX_Timeout=false;

// prototypes:
void switchPinAndMeasure(uint32_t pin, uint16_t* oldTimer, int16_t* result);
int16_t scaleplausServoFromPulse(uint16_t raw, int16_t old);
void PulseMeas(void);

static inline void handleEIC_RC(uint32_t pin)
{
	static uint16_t servo_in_OldTimer[4];
	eic_clear_int(pin==0?7:pin); // pin 0 is differently assigned!
	switchPinAndMeasure(pin==0?7:pin, &servo_in_OldTimer[pin], &servo_in_channel_raw[pin]);
}

static inline void handleEIC_PPM7(void)
{
	static uint16_t ppm_in_OldTimer = 0;
	static uint16_t channelNo;	
	eic_clear_int(7);
	
	
	uint16_t timer = AVR32_TC.channel[TIMERCHANNEL_RC].cv;
	
	uint16_t delta = timer - ppm_in_OldTimer;
	ppm_in_OldTimer = timer;
	
	if(delta < 100) return; // spike filter
	
	
	if(delta > RC_IN_TIMER_COUNT_1MS*3 || channelNo > 11) // > 3ms
	{	
		channelNo = 0;
	}		
	else	
	{	
		servo_in_channel_raw[channelNo++] = delta;
		RC_ResetTimeSinceLastRC(); // set timeout
	}		
		
}

static inline void handleEIC_dry(void)
{
	static uint16_t ppm_in_OldTimer = 0;
	static uint16_t channelNo;
	eic_clear_int(7);
	
	
	uint16_t timer = AVR32_TC.channel[TIMERCHANNEL_RC].cv;
	
	uint16_t delta = timer - ppm_in_OldTimer;
	ppm_in_OldTimer = timer;
	
	
	if( channelNo > 11) // > 3ms
	{
		channelNo = 0;
	}
	else
	{
		servo_in_channel_raw[channelNo++] = delta;
	}
	
}

__attribute__((__interrupt__))
static void eic_int_handler_RC1(void)
{
	if(RXType == RXType_Classic)
	{
		handleEIC_RC(0); 
	}
	else if (RXType == RXType_PPM7)
	{
		handleEIC_PPM7();
	}
	else if ( RXType == RXType_Unknown)
	{
		handleEIC_dry();
	}
	else
	{
		eic_clear_int(7); // just in case, there is another isr, which is not handled.
	}
}

__attribute__((__interrupt__))
static void eic_int_handler_RC2(void)
{
	handleEIC_RC(1);
}

__attribute__((__interrupt__))
static void eic_int_handler_RC3(void)
{
	handleEIC_RC(2);
}

__attribute__((__interrupt__))
static void eic_int_handler_RC4(void)
{
	handleEIC_RC(3);
}


void servo_in_init(uint8_t AlarmID)
{
	#if SKIPRCINIT == 1
	RXType = RXType_SUMD;
	#endif
	
	
	RC_in_timing_init();
	

	// Register the EIC interrupt handlers to the interrupt controller.
	INTC_register_interrupt(&eic_int_handler_RC1, AVR32_EIC_IRQ_7, AVR32_INTC_INTLEVEL_INT2);
	INTC_register_interrupt(&eic_int_handler_RC2, AVR32_EIC_IRQ_1, AVR32_INTC_INTLEVEL_INT2);
	INTC_register_interrupt(&eic_int_handler_RC3, AVR32_EIC_IRQ_2, AVR32_INTC_INTLEVEL_INT2);
	INTC_register_interrupt(&eic_int_handler_RC4, AVR32_EIC_IRQ_3, AVR32_INTC_INTLEVEL_INT2);

	// pre-wait (startup of RX)
	OS_WaitTicks(AlarmID,100); 
	
	// init for PPM7 for pulse meas.
	eic_enable_channel(7);
	eic_sw_edge_falling(7);

	do // retry as long as no signal comes in...
	{
		// wait for pulses
		OS_WaitTicks(AlarmID,300); // leave enough time for classic (12 frames a 20ms)
	
		uint32_t ppmValid=0,classicValid=0,serialValid=0;
	
		for(uint32_t i = 0; i< 12;i++)
		{
			// is it not a valid PPM channel?
			if(servo_in_channel_raw[i] > 1*RC_IN_TIMER_COUNT_1MS && servo_in_channel_raw[i] < 2*RC_IN_TIMER_COUNT_1MS)
			{
				// is PPM
				ppmValid++;
			}
			if(servo_in_channel_raw[i] < RC_IN_TIMER_COUNT_1MS*0.02 && servo_in_channel_raw[i] > 0)
			{
				// is serial
				serialValid ++;
			}
			if(servo_in_channel_raw[i] > RC_IN_TIMER_COUNT_1MS*15 /* && fixme compare for >21 ms not possible due to limited data type.*/ )
			{
				// is classic
				classicValid ++;
			}
		}

		if(ppmValid > serialValid && ppmValid > classicValid) // fixme add offsets!
		{
			RXType = RXType_PPM7;
		}
		else if(serialValid > ppmValid && serialValid > classicValid)
		{
			RXType = RXType_SerialUnknown;
		}
		else if(classicValid > serialValid && classicValid > ppmValid)
		{
			RXType = RXType_Classic;
		}
		else
		{
			// go around
		}
		
		wdt_clear(); // kick the dog
		
	} while (RXType == RXType_Unknown);
		

	switch(RXType)
	{
		case RXType_Unknown:
			break;
		case RXType_Classic:
			// pin function assigned in board_init.c
			eic_enable_channel(7);
			eic_enable_channel(1);
			eic_enable_channel(2);
			eic_enable_channel(3);
		break;
		case RXType_SPEKTRUM:
		case RXType_SUMD:
		case RXType_SerialUnknown:

			// enable module
			gpio_enable_module_pin(AVR32_PIN_PB03, 2 ); // USART1 - RXD "serial input RX")
		
			UsartRC_init(); // RC serial in as a block
			eic_disable_channel(7);
		break;
		case RXType_PPM7:
			// pin function assigned in board_init.c
			eic_disable_channel(7);
			#if PPMINVERTED == 0
			eic_sw_edge_rising(7); // fixme do variable
			#else
			eic_sw_edge_falling(7); // fixme do variable; this is only valid for FUTABA!
			#endif
			eic_enable_channel(7);
		break;
		default:
		break;
		
	}
	
}	



// measure pulse and return the result by reference.
// The result is not changed, if it was a rising edge!!!
// measurement range should be 0 for 1 ms and 10000 for 2 ms // only valid at 48MHz !!

void switchPinAndMeasure(uint32_t pin, uint16_t* oldTimer, int16_t* result)
{
	uint16_t res;
	
	if(eic_is_rising(pin))
	{
		*oldTimer = AVR32_TC.channel[TIMERCHANNEL_RC].cv; // remember start value
		eic_sw_edge_falling(pin);
	}			
	else
	{
		res = AVR32_TC.channel[TIMERCHANNEL_RC].cv;
		eic_sw_edge_rising(pin);
		res = res - *oldTimer; // overflow at 16bit !!
		
		
		if (res > RC_IN_TIMER_COUNT_1MS && res < 2*RC_IN_TIMER_COUNT_1MS) // signed, can't be other ;-)  
		{
			*result = res; // just store it, it will be scaled later.
			
			RC_ResetTimeSinceLastRC(); // set timeout
		}
		else
		{
			// leave result as it was...
		}
		
		
	}
}

int16_t scaleplausServoFromPulse(uint16_t raw, int16_t old)
{
	int32_t res;
	
	if (raw < RC_IN_TIMER_COUNT_1MS || raw > 2*RC_IN_TIMER_COUNT_1MS)
	{
		// failed measurement; return old.
		res = old;
	}
	else
	{
		res = ((((int32_t)raw - RC_IN_TIMER_COUNT_1MS)*666L)/100L) - SERVO_CENTER; // gives -5000..5000
	}
	
	return res;
}

// use only once per cycle!!
int servo_in_getCmd(int32_t* thr, int32_t* roll, int32_t* elev, int32_t* yaw)
{
	// fixme make commands the correct direction
	
	if(RC_GetTimeSinceLastRC()>100 ) // poll the timeout
	{
		// this is not desired... - failsafe
		if(*thr > 0) *thr-=5; // with 100Hz, it takes 2s from 1000 to 0 // fixme remove
		*roll = 0;
		*elev = 0;
		*yaw =  0;
		
		RX_Timeout = true;
		return 1;
	}

	if(RXType == RXType_Classic)
	{
		*thr = SERVO_CENTER + scaleplausServoFromPulse(servo_in_channel_raw[0],*thr-SERVO_CENTER);
		*roll =  scaleplausServoFromPulse(servo_in_channel_raw[1],*roll); // the first label paper is wrong with the number ;-)
		*elev =  scaleplausServoFromPulse(servo_in_channel_raw[2],*elev);
		*yaw =   scaleplausServoFromPulse(servo_in_channel_raw[3],*yaw);  // the first label paper is wrong with the number ;-)
	}
	else if(RXType == RXType_SPEKTRUM)
	{
		*thr = SERVO_CENTER + scaleplausServoFromSPEKTRUM(servo_in_channel_raw[0], *thr-SERVO_CENTER);
		*roll = scaleplausServoFromSPEKTRUM(servo_in_channel_raw[1],*roll); 
		*elev = scaleplausServoFromSPEKTRUM(servo_in_channel_raw[2],*elev);
		*yaw = scaleplausServoFromSPEKTRUM(servo_in_channel_raw[3],*yaw);
	}
	else if (RXType == RXType_SUMD)
	{
		*thr = SERVO_CENTER + scaleplausServoFromSUMD(servo_in_channel_raw[0],*thr-SERVO_CENTER);
		*roll =  scaleplausServoFromSUMD(servo_in_channel_raw[1],*roll);
		*elev =  scaleplausServoFromSUMD(servo_in_channel_raw[2],*elev);
		*yaw =   scaleplausServoFromSUMD(servo_in_channel_raw[3],*yaw);
	}
	else if(RXType == RXType_PPM7) // special for Futaba
	{
		*thr = SERVO_CENTER + scaleplausServoFromPulse(servo_in_channel_raw[2],*thr-SERVO_CENTER);
		*roll =  -scaleplausServoFromPulse(servo_in_channel_raw[0],*roll);
		*elev =  -scaleplausServoFromPulse(servo_in_channel_raw[1],*elev);
		*yaw =   -scaleplausServoFromPulse(servo_in_channel_raw[3],*yaw);
	}
	
	RX_Timeout = false;
	return 0;
}

int32_t servo_in_get_ext_channel(int ch, int32_t* value)
{
	if(ch <4 || ch >12 || RX_Timeout) // leave as it is, if out of range (also 0)
		return 1;
	
	if(RXType == RXType_Classic)
	{
		// no additional inputs - no values!
		return 2;
	}
	else if(RXType == RXType_SPEKTRUM)
	{
		*value = SERVO_CENTER*8/10 + scaleplausServoFromSPEKTRUM(servo_in_channel_raw[ch-1],0-SERVO_CENTER*8/10);
	}
	else if (RXType == RXType_SUMD)
	{
		*value = SERVO_CENTER*8/10 + scaleplausServoFromSUMD(servo_in_channel_raw[ch-1],0-SERVO_CENTER*8/10);
	}
	else if(RXType == RXType_PPM7)
	{
		// no inputs - no values!
		if(ch > 7)
		{	
			return 3;
		}
		*value = SERVO_CENTER*8/10 + scaleplausServoFromPulse(servo_in_channel_raw[ch-1],0-SERVO_CENTER*8/10);
	}
	else
	{
		return 3; // unknown RX type ??
	}

	return 0;
}

bool servo_in_get_ext_channel_switch(int ch, int SwP)
{
	if(ch <4 || ch >12|| RX_Timeout)
	return 0; // always return "OFF" on error or 0
	
	int32_t val = 0;
	servo_in_get_ext_channel(ch,&val);
	switch (SwP)
	{
		case 0:
			if(val >0 && val<= 3333)
				return 1;
			else
				return 0;
			break;
		case 1:
			if(val >3333 && val<= 6666)
				return 1;
			else
				return 0;
			break;
		case 2:
			if(val >6666 && val<= 10000)
				return 1;
			else
				return 0;
			break;
		default:
				return 0;
			break;
	}

}


bool servo_in_Start(int32_t cmd_thr, int32_t cmd_roll, int32_t cmd_elev, int32_t cmd_yaw)
{
	static uint8_t cnt=0;
	
	// wait for all sticks being in idle condition
	if((cmd_thr < 1500) && (abs(cmd_yaw) < 1000) && (abs(cmd_elev) < 1000) && (abs(cmd_roll) < 1000))
	{
		if (cnt++ >=10)
		{
			return true;
		}
	}
	else
	{
		cnt = 0;
	}
	return false;
}