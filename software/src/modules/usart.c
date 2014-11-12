/*
 * USART.c
 *
 * Created: 12.02.2014 21:59:19
 *
 * (c) 2014 by Fabian Huslik
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
#include "types.h"
#include "usart.h"

typedef struct usart_mgmt_tag
{
	volatile avr32_usart_t* ptr;
	volatile uint32_t RX_cnt;
	volatile uint32_t TX_cnt;
	uint8_t* pRXbuf;
	uint32_t RXbufsize;
	//uint32_t RXbufptr;
	uint8_t* TXptr;
	vfpuc_t bytehandler;
	vfpv_t blockhandler;
		
}usart_mgmt_t;

static usart_mgmt_t myUsarts[3];

int UsartIntBlockHandler( int ch);
void USARTs_init(void);
__attribute__((__interrupt__)) static void usart0_int_handler( void ); 
__attribute__((__interrupt__)) static void usart1_int_handler( void ); 
__attribute__((__interrupt__)) static void usart2_int_handler( void ); 

void USARTs_init(void)
{
	myUsarts[0].ptr = &AVR32_USART0;
	myUsarts[1].ptr = &AVR32_USART1;
	myUsarts[2].ptr = &AVR32_USART2;
		
	INTC_register_interrupt(&usart0_int_handler, AVR32_USART0_IRQ, AVR32_INTC_INT1);
	INTC_register_interrupt(&usart1_int_handler, AVR32_USART1_IRQ, AVR32_INTC_INT1);
	INTC_register_interrupt(&usart2_int_handler, AVR32_USART2_IRQ, AVR32_INTC_INT1);
}


void USART_init(uint32_t number, uint32_t baudrate , uint8_t* RXbuffer, uint32_t RXBufferSize, vfpuc_t RX_byte_handler, vfpv_t RX_block_handler)
{
	USARTs_init(); // necessary only once, but not important, if executed twice or more times.

	myUsarts[number].pRXbuf = RXbuffer;
	myUsarts[number].RXbufsize = RXBufferSize;

	// USART options.
	static usart_options_t USART_OPTIONS =
	{
		.baudrate = 0,
		.charlength = 8,
		.paritytype = USART_NO_PARITY,
		.stopbits = USART_1_STOPBIT,
		.channelmode = USART_NORMAL_CHMODE
	};
	USART_OPTIONS.baudrate = baudrate;

	// Initialize USART in RS232 mode.
	usart_init_rs232(myUsarts[number].ptr, &USART_OPTIONS, BOARD_SYS_HZ);

	myUsarts[number].bytehandler = RX_byte_handler;
	myUsarts[number].blockhandler = RX_block_handler;
	
	// Enable USART Rx interrupt.
	myUsarts[number].ptr->CR.rststa = 1;// reset flags
	myUsarts[number].ptr->IER.rxrdy = 1;
	//	AVR32_USART2.IER.txrdy = 1;
	//	AVR32_USART2.IER.ovre = 1;
	myUsarts[number].ptr->IER.frame = 1;

	myUsarts[number].ptr->IER.timeout = RX_block_handler==NULL?0:1; // timeout interrupt enable, if handler linked
	myUsarts[number].ptr->RTOR.to = 16;
		
	myUsarts[number].ptr->CR.rxen = 1; // RX interrupt enable
	myUsarts[number].ptr->CR.txen = 1;
}

__attribute__((__interrupt__)) static void usart0_int_handler( void ) 
{
	UsartIntBlockHandler(0);
}

__attribute__((__interrupt__)) static void usart1_int_handler( void )
{
	UsartIntBlockHandler(1);
}

__attribute__((__interrupt__)) static void usart2_int_handler( void )
{
	UsartIntBlockHandler(2);
}


int UsartIntBlockHandler( int ch)
{
	avr32_usart_csr_t stat;

	stat = myUsarts[ch].ptr->CSR; // read status

	if(stat.ovre) // overrun
	{
		myUsarts[ch].RX_cnt = 0;
		myUsarts[ch].ptr->CR.rststa = 1; // reset error status bits
		myUsarts[ch].ptr->CR.sttto = 1; // deactivate timeout till next (first) byte	return RXbufptr;
		return 4;
	}
	if(stat.frame) // frame error
	{
		myUsarts[ch].RX_cnt = 0;
		myUsarts[ch].ptr->CR.rststa = 1; // reset error status bits
		myUsarts[ch].ptr->CR.sttto = 1; // deactivate timeout till next (first) byte	return RXbufptr;
		return 5;
	}

	if(stat.timeout)
	{
		myUsarts[ch].ptr->CR.sttto = 1; // deactivate timeout till next (first) byte
		if(myUsarts[ch].blockhandler != NULL)
		{
			myUsarts[ch].blockhandler();
		}
		
		return 0; // the block is now finished
	}
	
	if(stat.rxrdy)
	{
		uint8_t c = myUsarts[ch].ptr->RHR.rxchr;
		
		// read char (implicitly activates timeout)
		if(myUsarts[ch].bytehandler != NULL)
		{
			myUsarts[ch].bytehandler(c); // fixme with parameter "the byte"
		}
		
		if(myUsarts[ch].RXbufsize == 0 || myUsarts[ch].pRXbuf == NULL )
		{
			myUsarts[ch].RX_cnt = 0;
			myUsarts[ch].ptr->CR.rststa = 1; // reset error status bits
			myUsarts[ch].ptr->CR.sttto = 1; // deactivate timeout till next (first) byte	return RXbufptr;
			return 7;
		}
		
		
		myUsarts[ch].pRXbuf[myUsarts[ch].RX_cnt++] = c;

		if(( myUsarts[ch].RX_cnt ) >= myUsarts[ch].RXbufsize) // overflow?
		{
			myUsarts[ch].RX_cnt = 0;
			// return error?
		}
	}

	if(stat.txempty)
	{
		// transmit next char - if any
		if(myUsarts[ch].TX_cnt >0)
		{
			myUsarts[ch].ptr->THR.txchr = *myUsarts[ch].TXptr++;
			myUsarts[ch].TX_cnt--;
		}
		else
		 myUsarts[ch].ptr->IDR.txempty = 1;
	}

	return 1; // 0 on block finished, 1 on busy, all other are real errors.
}


void USART_Send(int ch, uint8_t* stuff,uint32_t len) // the buffer has to be static or living long enough!
{
	if(myUsarts[ch].TX_cnt >0)
	{
		asm("breakpoint");//fixme debug warum kommen wir hier vorbei??? // 
		return;
	}
	
	myUsarts[ch].TXptr = stuff;
	myUsarts[ch].TX_cnt = len;
	
	// enable TX int
	myUsarts[ch].ptr->IER.txempty = 1;
}

void USART_putchar(int ch, uint8_t c)
{
	usart_putchar(myUsarts[ch].ptr, c);

}


uint8_t* USART_GetRxData(int ch, uint32_t* len) // the length is reset after this! // fixme perform a copy to a provided buffer!
{
	*len = myUsarts[ch].RX_cnt;
	myUsarts[ch].RX_cnt = 0; // no double buffer !
	return myUsarts[ch].pRXbuf;
}