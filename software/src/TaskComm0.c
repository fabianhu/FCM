/*
 * TaskComm0.c
 * using com0 (Upper port at FCM)
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
#include "modules/types.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include <string.h>
#include "modules/fcmcp.h"
#include "menu/menu.h"
#include "menu/menu_variant.h"
#include "modules/MAVlink.h"
#include "config.h"
#include "modules/usart.h"
#include "modules/hottv4.h"
#include "menu/menu.h"
#include "modules/hottv4.h"
#include "modules/HoTTInterface.h"
#include "config.h"

void TaskComm0(void);
void usart_write_line_BTM(const char *string);
void delay_Comm0(uint32_t ms);
void BluetoothConfigOnce(void);
void FCMCPReadyHandler(void);  // callback from ISR
void MAVLinkReadyHandler(void); // callback from ISR

#if ( COM0_FCMCP == 1)||(COM0_MAVLINK == 1)
static uint8_t COM0RXbuf[100];
#endif

void TaskComm0(void)
{
	if(myPar.NoConfigBTM222atStart.sValue == 0)
	{
		BluetoothConfigOnce();
		delay_Comm0(500);
	}
	
#if COM0_FCMCP == 1
	menu_init();
	USART_init(0,115200,COM0RXbuf,sizeof(COM0RXbuf),NULL,FCMCPReadyHandler);
#elif COM0_HOTT == 1
	static uint32_t a;
	USART_init(0,19200,NULL,0,HoTT_byte_handler,NULL); // no buffer, bytes are buffered in special buffer (why ever ...)
	menu_init();
	hott_init();
#elif COM0_MAVLINK == 1
	MAV_init();
	USART_init(0,115200,COM0RXbuf,sizeof(COM0RXbuf),NULL,MAVLinkReadyHandler);
#else
	#error "select a comm protocol!"
#endif

	while(1)
	{
#if COM0_FCMCP == 1
		OS_WaitTicks(OSALM_COMM0WAIT,1000); // just idle around... 
#elif COM0_HOTT == 1
		OS_WaitTicks(OSALM_COMM0WAIT,1);
		a++;
		hott_serial_scheduler( a * 1000 );
#elif COM0_MAVLINK == 1
		MAV_Process();
#endif
		
	}
}

void FCMCPReadyHandler(void)  // callback from ISR
{
	uint8_t* ptr;
	uint32_t len;
	ptr = USART_GetRxData(0,&len);
	fcmcp_analyzePacket((char*)ptr);
}		

void MAVLinkReadyHandler(void) // callback from ISR
{
	uint8_t* ptr;
	uint32_t len;
	ptr = USART_GetRxData(0,&len);
	MAV_CopyPacket(ptr,len);
}


void delay_Comm0(uint32_t ms)
{
	OS_WaitTicks(OSALM_COMM0WAIT,ms);
}
// call only in TaskComm0 !!!
void BluetoothConfigOnce(void)
{
	USART_init(0,19200,NULL,0,NULL,NULL);
	
	// bluetooth init
	usart_write_line((&AVR32_USART0),"+++"); // get module out of comm mode.
	delay_Comm0(1100); // required 1000ms guard time
	usart_write_line((&AVR32_USART0),"+++"); // get module out of comm mode.
	delay_Comm0(1100); // required 1000ms guard time
	usart_write_line_BTM("AT"); // wake up (expect error)
	delay_Comm0(500);
	
	usart_write_line_BTM("ATZ0"); // reset module to default // only necessary once!
	delay_Comm0(3500);					// yes, it takes soo long!
	usart_write_line_BTM("ATC0"); // switch off flow control (needed once only, not affected by ATZ0 = reset all)
	delay_Comm0(3500); 
	usart_write_line_BTM("ATN=FCM");
	delay_Comm0(500); 
	
	usart_write_line_BTM("ATE0"); // echo off
	delay_Comm0(300);
	usart_write_line_BTM("ATL5"); // 115200bps
	delay_Comm0(300);
}


void usart_write_line_BTM(const char *string)
{
	while (*string != '\0')
	{
		USART_putchar(0, *string++);
		delay_Comm0(50); // 30 to 50 ms from data sheet
	}
	USART_putchar(0, 0x0d);
}
