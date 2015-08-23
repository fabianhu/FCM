/*
 * fcmcp.c FCM Communication Protocol
 *
 * Created: 28.01.2013 22:29:53
 *
 * (c) 2013-2015 by Fabian Huslik
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
#include "fcmcp.h"
#include <string.h>
#include "types.h"
#include "../version.h"
#include "hottv4.h"
#include "../menu/menu.h"
#include "../menu/menu_variant.h"
#include "TaskMenu.h"
#include "usart.h"

void CheckDelay( char* buf );

static fcmcp_state_t fcmcp_state;
uint8_t fcmcp_TXdelay = 0;

uint8_t fcmcp_buf[HOTT_TEXTMODE_MSG_TEXT_LEN+10] = VERSION_STRING;

void CheckDelay( char* buf ) 
{
	if(strncmp(&buf[6],"~~~",3) == 0)
		fcmcp_TXdelay = 0;
	else
		if(strncmp(&buf[7],"~~~",3) == 0)
			fcmcp_TXdelay = buf[6];
		else
			fcmcp_TXdelay = 255;
}

void fcmcp_analyzePacket(char* buf) // in ISR
{

	if ( strncmp(buf,"---FCM~~~",9) == 0)
	{
		fcmcp_state = fcmcp_stream_idle; // prevent that the FCM overflows this own answers
		// send answer "FCM Version"
		strncpy((char*)&fcmcp_buf[0],"---FCM",6);
		fcmcp_buf[6]= VERSION_MAJOR;
		fcmcp_buf[7]= VERSION_MINOR;
		fcmcp_buf[8]= VERSION_BUILDH;
		fcmcp_buf[9]= VERSION_BUILDL;
		fcmcp_buf[10]= (MENUESIZE>>8);
		fcmcp_buf[11]= (MENUESIZE&0xff);
		strncpy((char*)&fcmcp_buf[12],"~~~",3);
		USART_Send(0,fcmcp_buf,9+4+2);  // fixme modularize
		
		return;
	}	
	
	if ( strncmp(buf,"---STD",6) == 0)
	{
		CheckDelay(buf);
		// start streaming data
		fcmcp_state = fcmcp_stream_conv;
		return;
	}

	if ( strncmp(buf,"---STQ",6) == 0)
	{
		CheckDelay(buf);
		// start streaming data
		fcmcp_state = fcmcp_stream_quat;
		return;
	}

	if ( strncmp(buf,"---STG",6) == 0)
	{
		CheckDelay(buf);
		// start streaming data
		fcmcp_state = fcmcp_stream_GPS;
		return;
	}
	
	if ( strncmp(buf,"---STS~~~",9) == 0)
	{
		// stop streaming data
		fcmcp_state = fcmcp_stream_idle;
		return;
	}
	
	if ( strncmp(buf,"---MNU",6) == 0 )
	{
		fcmcp_state = fcmcp_stream_menu;
		// call menue with parameter buf[6]
		// fire the menu task here
		strncpy((char*)&fcmcp_buf[0],"---TXT",6);
		TMNU_SetPtr(&fcmcp_buf[6]);
		TMNU_TriggerMenu(buf[6],1); // hand over the key command
		
		return;
	}
	if ( strncmp(buf,"---BOOT~~~",10) == 0 )
	{
		// jump into bootloader
		AVR32_PM.gplp[0] = 0xB00710AD; // magic word for BL "bootload"
		
		wdt_opt_t wdopt;
		wdopt.us_timeout_period = 1000000; // this is the only one parameter. // leave time for the app to re-init even with slow clock.
		wdt_disable();
		wdt_enable(&wdopt);
		cpu_irq_disable();
		while(1)
		{
			//
		}

		
	}
	
	if ( strncmp(buf,"---PARL~~~",10) == 0 )
	{
		// output of parameter list
		fcmcp_state = fcmcp_stream_parl; // done by comm task
	}
	
	if ( strncmp(buf,"---PARV",7) == 0 )
	{
		// set parameter value
		uint8_t id = buf[7];
		int16_t val = buf[8]<<8|buf[9];
		menue_setParVal(id,val);
	}
	
}

fcmcp_state_t fcmcp_getStreamState(void)
{
	return fcmcp_state;
}

uint8_t fcmcp_getTXDelay(void)
{
	return fcmcp_TXdelay;
}



void fcmcp_setStreamState(fcmcp_state_t e)
{
	fcmcp_state = e;
}

void fcmcp_send(uint8_t* pData, uint32_t len)
{
	USART_Send(0,pData,len); // fixme modularize
}