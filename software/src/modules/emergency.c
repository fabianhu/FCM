/*
 * emergency.c
 *
 * Created: 11.01.2013 22:01:26
 *
 * (c) 2013-2014 by Fabian Huslik
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

#include "asf.h"
#include "servo_out.h"
#include "TaskControl.h"
#include "TaskLED.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "emergency.h"
#include "config.h"

volatile uint32_t reason=0;

void emstop(uint32_t _reason)
{
	#if NOEMSTOP == 1
		#warning you are crazy!
	return;
	#endif
	
	// stop all ISRs
	OS_DISABLEALLINTERRUPTS;
	
	// stop all PWM
	servo_out_set(0,0);
	servo_out_set(1,0);
	servo_out_set(2,0);
	servo_out_set(3,0);
	servo_out_set(4,0);
	servo_out_set(5,0);
	
	reason = _reason;

	asm("breakpoint");
	
	// switch off all LED
	LED_RED_ON;
	LED_GREEN_OFF;
	LED_BLUE_OFF;
	
	delay_init(BOARD_SYS_HZ);
	
	while(1)
	{
		// todo maybe reset instead of sitting here???
		
		for(uint32_t i = 0; i< _reason;i++)
		{
			LED_BLUE_ON;
			LED_GREEN_ON;
			delay_ms(300);
			LED_BLUE_OFF;
			LED_GREEN_OFF;
			delay_ms(300);
		}
		delay_ms(1000);
	}
	
	Reset_CPU();
}