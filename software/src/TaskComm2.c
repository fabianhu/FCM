/*
 * TaskComm2.c
 * using com2 (Bottom side)
 *
 * Created: 06.09.2012 09:44:55
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
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/types.h"
#include "modules/usart.h"
#include "modules/GPS.h"


void TaskComm2(void);
void GPS_byte_handler(uint8_t c);

static uint8_t GPSRXbuf[100];

void TaskComm2(void)
{
	USART_init(2,115200,GPSRXbuf,sizeof(GPSRXbuf),GPS_byte_handler,NULL);
	
	while(1)
	{
		OS_WaitTicks(OSALM_COMM2WAIT,1000);
		
	}
	
}


void GPS_byte_handler(uint8_t c)
{
	GPS_NMEA_newFrame((char)c);
}