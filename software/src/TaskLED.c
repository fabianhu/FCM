/*
 * TaskLED.c
 * using led
 *
 * Created: 09.10.2012 19:49:13
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
#include "TaskControl.h"
#include "TaskLED.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"

#define LEDWAIT(x) OS_WaitTicks(OSALM_LEDWAIT,x);

void TaskLED(void);

// module static vars
FlightState_t myFlightstate; 


void TaskLED(void)
{

	while(1)
	{
		LED_RED_OFF;
		LED_GREEN_OFF;
		LED_BLUE_OFF;
		switch(myFlightstate)
		{
			case FS_preinit:
				// scroll
				LED_RED_ON;
				LED_GREEN_ON;
				LEDWAIT(150);
				LED_GREEN_OFF;
				LED_BLUE_ON;
				LEDWAIT(150);
				break;
			
			case  FS_init:
				// scroll
				LED_RED_ON;
				LEDWAIT(100);
				LED_RED_OFF;
				LED_GREEN_ON;
				LEDWAIT(100);
				LED_GREEN_OFF;
				LED_BLUE_ON;
				LEDWAIT(100);
				break;
			case  FS_init_sensor_zero:
				// fast blink blue
				LEDWAIT(150);
				LED_BLUE_ON;
				LEDWAIT(150);
				break;			
			case  FS_idle:
				// slow flash green
				LEDWAIT(900);
				LED_GREEN_ON;
				LEDWAIT(100);
				break;
			case  FS_autoarm:
				// blue/green alternating
				LED_BLUE_ON;
				LEDWAIT(333);
				LED_BLUE_OFF;
				LED_GREEN_ON;
				LEDWAIT(333);
				break;		
			case  FS_fly:
				// steady blue
				LED_BLUE_ON;
				LEDWAIT(300);
				break;
			case FS_error:
				LEDWAIT(350);
				LED_RED_ON;
				LEDWAIT(50);
				break;
			default:
				LEDWAIT(100);
				LED_RED_ON;
				LEDWAIT(100);
				break;

		}
		
	}
}

static uint32_t lastStateChangeTime;

// interfacing functions
void LED_SetFlightstate(FlightState_t fs) // includes remembering last state change time?
{
	lastStateChangeTime = OS_GetTicks();
	myFlightstate = fs;	
}

FlightState_t LED_GetFlightstate(void)
{
	return myFlightstate;
}

uint32_t LED_GetLastFlightstateChanged(void)
{
	return OS_GetTicks() - lastStateChangeTime;
}