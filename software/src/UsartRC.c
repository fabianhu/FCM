/*
 * UsartRC.c
 *
 * Created: 28.07.2012 14:23:05
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


#include<asf.h>
#include "UsartRC.h"
#include "servo_in.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/SUMD.h"
#include "modules/SPEKTRUM.h"
#include "modules/types.h"
#include "modules/usart.h"


void RC_int_block_handler( void );
void AnalyzeSerialRXFrame(int16_t* ServoValues, volatile uint8_t* buffer, uint32_t* len);

#define RC_SERBUFSIZE 40 // 37 should be enough for 16ch Graupner SUMD ; 16 for SPEKTRUM 6ch

uint32_t RX_serial_buf_cnt; // number of bytes
uint8_t RX_serial_buf[RC_SERBUFSIZE];

uint32_t debug_RXINTERR,debug_RXERR;
	
void RC_int_block_handler( void ) // SUMD data frame handler
{
	uint32_t len;
	volatile uint8_t* data = USART_GetRxData(1,&len);
		// analyze buffer and push raw values into array.
		AnalyzeSerialRXFrame(servo_in_channel_raw,data,&len );
		
}


void UsartRC_init( void ) // SUMD
{
	USART_init(1,115200,RX_serial_buf,sizeof(RX_serial_buf),NULL,RC_int_block_handler);
	

	// always on.
	gpio_clr_gpio_pin(AVR32_PIN_PA30); // switch on 3.3V power for RX (on spektrum placement option, for others it does not matter)
	
	
}


uint32_t RC_LastTime;


void AnalyzeSerialRXFrame(int16_t* ServoValues, volatile uint8_t* buffer, uint32_t* len)
{
	uint8_t ret;
	if(RXType == RXType_SerialUnknown)
	{
		// try and set
		ret = SUMD_ConvertToServos(ServoValues,buffer); // fixme outside of ISR maybe?
		if(ret != 0)
		{
			// no error occurred
			RXType = RXType_SUMD;
		}
		else
		{
			if(*len == 16)
			{
				ret = SPEKTRUM_ConvertToServos(ServoValues,buffer); // fixme outside of ISR maybe?
				if(ret != 0) 
				{
					// no error occurred
					RXType = RXType_SPEKTRUM;
				}
			}

			else
			{
				// retry... or other format...
			}
		}
		
	}
	
	
	if(RXType == RXType_SPEKTRUM)
	{
		ret = SPEKTRUM_ConvertToServos(ServoValues,buffer); // todo outside of ISR maybe?
		if(ret != 0)
		{
			RC_LastTime =  OS_GetTicks();
		}
		else
		{
			debug_RXERR++;
		}
	}
	
	if(RXType == RXType_SUMD)
	{
		ret = SUMD_ConvertToServos(ServoValues,buffer); // todo outside of ISR maybe?
		if(ret != 0)
		{
			RC_LastTime =  OS_GetTicks();
		}
		else
		{
			debug_RXERR++;
		}
	}
	
	*len=0;

}

uint32_t RC_GetTimeSinceLastRC(void)
{
	return  OS_GetTicks() - RC_LastTime;
}

void RC_ResetTimeSinceLastRC(void)
{
	RC_LastTime = OS_GetTicks();
}