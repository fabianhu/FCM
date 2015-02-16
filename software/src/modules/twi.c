/*
 * twi.c
 *
 * Created: 11.10.2012 20:09:49
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
 
 todo: PDCA - DMA supported read instead of ISR based read.
 see Avrfreaks thread: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=126730
 
 */ 

#include "asf.h"
#include "types.h"
#include "twi.h"
#include "../FabOS_config.h"
#include "../FabOS/FabOS.h"
#include <string.h> // for memcpy
#include "emergency.h"


static int twi_set_speed(volatile avr32_twi_t *twi, unsigned int speed, unsigned long pba_hz);

__attribute__((__interrupt__)) static void TWI_interrupt_handler(void);


volatile uint8_t* TWI_ISR_RXpbuf;
volatile uint8_t TWI_ISR_RXcnt = 0;

volatile uint8_t* TWI_ISR_TXpbuf;
volatile uint8_t TWI_ISR_TXcnt = 0;

vfpv_t twi_ready_fp = NULL;

void TWI_RegisterReadyHandler(vfpv_t fp)
{
	twi_ready_fp = fp;
}

uint32_t debug_TWI_noTxComp=0;
uint32_t debug_TWI_TxComp=0;

__attribute__((__interrupt__))
static void TWI_interrupt_handler(void)
{
	avr32_twi_sr_t sr;
	*((uint32_t*)&sr) = AVR32_TWI.sr;
	
	if(sr.rxrdy) // a single character is ready to be picked up.
	{
		*TWI_ISR_RXpbuf++ = AVR32_TWI.RHR.rxdata;
		TWI_ISR_RXcnt--;
		if(TWI_ISR_RXcnt == 1)
		{
			// next is last byte
			AVR32_TWI.CR.stop = 1;// set stop condition
		}
		else if(TWI_ISR_RXcnt == 0)
		{
			if(sr.txcomp == 0)
			{
				AVR32_TWI.CR.stop = 1;// set stop condition
				debug_TWI_noTxComp++;
			}
			else
			{
				debug_TWI_TxComp++;
			}
			
			
			
			// this is last byte, disable isr and stuff.
			AVR32_TWI.IDR.rxrdy = 1; //disable end of receive interrupt
			AVR32_TWI.IDR.nack = 1; // disable NACK isr
			
			// callback
			if(twi_ready_fp != NULL)
				twi_ready_fp();
		}
		else
		{
			// rx byte in between
		}

	}
	if(sr.nack)
	{
		asm("breakpoint"); // fixme what to do
		TWI_ISR_RXcnt = 0;
		TWI_ISR_TXcnt = 0;		
	}
	if(sr.txrdy && TWI_ISR_TXcnt>0) 
	{
		AVR32_TWI.THR.txdata = *TWI_ISR_TXpbuf++ ;
		TWI_ISR_TXcnt--;
		
		if(TWI_ISR_TXcnt == 0)
		{
			// this is last byte
			// stop condition is done automatically...
		}
		else
		{
			// byte in between
		}
	}
	if(sr.txcomp) // TX is now really finished and acknowledged.
	{
		if(TWI_ISR_TXcnt>0) 
		{ 
			asm("breakpoint");
		}			
		AVR32_TWI.IDR.nack = 1; // disable NACK isr
		AVR32_TWI.IDR.txcomp = 1; // disable TXCOMP isr
		AVR32_TWI.IDR.txrdy = 1; //disable tx empty isr
		// callback
		if(twi_ready_fp != NULL)
			twi_ready_fp();
	}
}


void TWI_write_ISR(uint8_t device, uint8_t addr, uint8_t len, uint8_t* src) 
{
	if(TWI_ISR_RXcnt != 0 || TWI_ISR_TXcnt != 0)
	{
		//asm("breakpoint"); // fixme handle it
		return; // still busy!
	}
	TWI_ISR_TXpbuf = src;
	TWI_ISR_TXcnt = len;
	
	AVR32_TWI.MMR.dadr = device;
	AVR32_TWI.MMR.mread = 0; // write direction
	AVR32_TWI.MMR.iadrsz = 1; // length of sub-address in bytes (0..3)
	
	AVR32_TWI.IADR.iadr = addr;
	
	AVR32_TWI.CR.stop = 0; // set automatically, if nothing is put into THR
	
	OS_DISABLEALLINTERRUPTS; // prevent ISRs from going off before sending started.
	
	AVR32_TWI.IER.txcomp = 1; // the transfer is complete and acknowledged.
	AVR32_TWI.IER.txrdy = 1; // fill the next byte into THR
	AVR32_TWI.IER.nack = 1; // naaaaack

	// start the transfer NOW
	AVR32_TWI.THR.txdata = *TWI_ISR_TXpbuf++;
	TWI_ISR_TXcnt--;
	OS_ENABLEALLINTERRUPTS;
}

twitrnsf_t debug_TWI_lastTrnsf;

int32_t debug_TWI_CountOfMisReads=0; // used for dirty hack to go to acro mode.

uint8_t debug_dummy;
avr32_twi_sr_t debug_sr;

void TWI_read_ISR(uint8_t device, uint8_t addr, uint8_t len, uint8_t* dest) 
{

	*((uint32_t*)&debug_sr) = AVR32_TWI.sr;
	

	debug_TWI_lastTrnsf.addr = addr;
	debug_TWI_lastTrnsf.device = device;
	debug_TWI_lastTrnsf.len = len;
	debug_TWI_lastTrnsf.lastTime = OS_GetTicks();

	debug_dummy = AVR32_TWI.RHR.rxdata; // just for fun...
	if(TWI_ISR_RXcnt != 0 || TWI_ISR_TXcnt != 0 || !debug_sr.txcomp) 
	{
		// fixme hier aufräumen / resetten // Fehler an "Anforderer" mitteilen = Status umsetzen. //AVR32_TWI.CR.stop = 1;
		//asm("breakpoint"); // WTF does the Interrupt not fire ???????????????????????????????????????????????????????????????
		
		/*
		Findings:
		The condition is entered mostly during debugging
		Even a reset of the CPU does not resolve the condition.
		After power off, everything works as normal.
		
		There is NO way to reset the TWI engine with CPU powered up.
		
		*/
		
		
		
		debug_TWI_CountOfMisReads++;
		
		AVR32_TWI.CR.swrst = 1;
		TWI_init();
		
		//emstop(4); // fixme attention! 
		return; // still busy!
	}		
	
	TWI_ISR_RXpbuf = dest;
	TWI_ISR_RXcnt = len;
	
	
	AVR32_TWI.MMR.dadr = device;
	AVR32_TWI.MMR.iadrsz = 1; // length of sub-address in bytes (0..3)

	AVR32_TWI.MMR.mread = 1; // read direction
		
	AVR32_TWI.IADR.iadr = addr;	
	
	OS_DISABLEALLINTERRUPTS;
	
	AVR32_TWI.IER.rxrdy = 1; // enable end of receive interrupt
	AVR32_TWI.IER.nack = 1; // enable NACK isr

	// start the transfer
	if(len == 1)
	{
		AVR32_TWI.cr = AVR32_TWI_CR_START_MASK | AVR32_TWI_CR_STOP_MASK; 
	}		
	else
		AVR32_TWI.cr = AVR32_TWI_CR_START_MASK; // not for 1st byte

	OS_ENABLEALLINTERRUPTS;

}

void TWI_init(void)
{
	sysclk_enable_pba_module(SYSCLK_TWI);
	
	OS_DISABLEALLINTERRUPTS;
	// Reset TWI
	AVR32_TWI.CR.swrst = 1;
	AVR32_TWI.rhr; // dummy read for errata
	// Dummy read in SR
	AVR32_TWI.sr;

	// Register TWI handler on level 1
	INTC_register_interrupt( &TWI_interrupt_handler, AVR32_TWI_IRQ, AVR32_INTC_INT3);
	OS_ENABLEALLINTERRUPTS;	
	// Select the speed
	twi_set_speed(&AVR32_TWI, 400000, BOARD_SYS_HZ);
	
	// disable slave mode
	AVR32_TWI.CR.svdis = 1; 
	// enable master mode
	AVR32_TWI.CR.msen = 1;
	

	/*
	The following registers have to be programmed before entering Master mode:
	1. DADR (+ IADRSZ + IADR if a 10 bit device is addressed): The device address is used
	to access slave devices in read or write mode.
	2. CKDIV + CHDIV + CLDIV: Determines clock waveform Thigh and Tlow.
	3. SVDIS: Disable the slave mode.
	4. MSEN: Enable the master mode.
	*/

}

/*! \brief Set the twi bus speed in conjunction with the clock frequency
 *
 * \param twi    Base address of the TWI (i.e. &AVR32_TWI).
 * \param speed  The desired twi bus speed
 * \param pba_hz The current running PBA clock frequency
 * \return TWI_SUCCESS
 */
static int twi_set_speed(volatile avr32_twi_t *twi, unsigned int speed, unsigned long pba_hz)
{
  unsigned int ckdiv = 0;
  unsigned int c_lh_div;

  c_lh_div = pba_hz / (speed * 2) - 4;

  // cldiv must fit in 8 bits, ckdiv must fit in 3 bits
  while ((c_lh_div > 0xFF) && (ckdiv < 0x7))
  {
    // increase clock divider
    ckdiv++;
    // divide cldiv value
    c_lh_div /= 2;
  }

  // set clock waveform generator register
  twi->cwgr = ((c_lh_div << AVR32_TWI_CWGR_CLDIV_OFFSET) |
              (c_lh_div << AVR32_TWI_CWGR_CHDIV_OFFSET) |
              (ckdiv << AVR32_TWI_CWGR_CKDIV_OFFSET));

  return 0;
}


