/*
 * UartHoTT.c
 *
 * Created: 02.11.2012 23:12:48
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

#include<asf.h>
#include "HoTTInterface.h"
#include "HoTTv4.h"
#include "servo_in.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/SUMD.h"
#include "config.h"
#include "modules/GPS.h"
#include "modules/types.h"
#include "modules/usart.h"

static uint8_t hottRXbuf[100];

static volatile uint32_t hott_fill; // points to the next write location (0 on empty)
static volatile uint32_t hott_read; // next byte to be read (if hott_fill is bigger !)

void HoTT_byte_handler( uint8_t c ) // hott byte handler
{
	hottRXbuf[hott_fill++] = c;

	if(hott_fill > sizeof( hottRXbuf )) // overflow?
	{
		HoTTFlushBuffer();
	}
}

void HoTTSendByte( uint8_t c )
{
	USART_putchar(0,c);
}

// return available bytes in rx buffer
uint8_t HoTTGetAvailable( void )
{
	return hott_fill - hott_read;
}

// get byte from buffer
uint8_t HoTTRecByte( void )
{
	if(hott_fill <= hott_read)
	return 0; //read too far...
	else
	return hottRXbuf[hott_read++];
}

// flush buffer
void HoTTFlushBuffer( void )
{
	hott_read = 0;
	hott_fill = 0;
}

void HoTTEnableReceiverMode( void )
{
	// disable tx line driver
	AVR32_USART0.CR.txdis = 1; // fixme fixed to usart 0
	AVR32_USART0.CR.rsttx = 1;
	asm("nop");
	gpio_enable_gpio_pin (AVR32_USART0_TXD_0_0_PIN); // output driver is disabled there!!
	AVR32_USART0.CR.rxen = 1;

}

void HoTTEnableTransmitterMode( void )
{
	// enable tx line and disable rx (receiving own stuff...)
	AVR32_USART0.CR.rxdis = 1;
	AVR32_USART0.CR.rstrx = 1;
	asm("nop");
	gpio_enable_module_pin( AVR32_USART0_TXD_0_0_PIN, AVR32_USART0_TXD_0_0_FUNCTION );
	AVR32_USART0.CR.txen = 1;
}
