/*
 * FCMCP_BL.c
 *
 * Created: 22.09.2013 23:39:44
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



#include <asf.h>
#include "fcmcp_bl.h"
#include "version.h"
#include <string.h>

void fcmcp_answer(uint8_t* buf, uint32_t len);
void fcmcp_answerByte(uint8_t by);
bool fcmcp_IsAnswerPossible(void);

void fcmcp_answer(uint8_t* buf, uint32_t len) // send some bytes up.
{
	udi_cdc_write_buf(buf,len);
}

void fcmcp_answerByte(uint8_t by) // send some bytes up - stream version for flash readout
{
	udi_cdc_putc(by);
}

bool fcmcp_IsAnswerPossible(void) // is it possible to send now?
{
	if(udi_cdc_get_free_tx_buffer()>0)
	return true;
	else
	return false;
}


uint8_t fcmcp_responsebuf[20]; // for short answers only...
uint8_t FlashWriteMode = 0;
uint32_t FlashWritePos = 0;
uint32_t FlashWriteRemaining = 0;

extern void StartApplication(void);

void fcmcp_analyzePacket(uint8_t* _buf,uint32_t _len)
{
	static uint32_t FlashAdr,FlashLen; // fixme debug static
	
	if(FlashWriteMode == 1)
	{
		// fixme: what if length > remaining ??
		
		
		// write byte(s) directly to flash
		flashc_memcpy((void*)FlashWritePos,_buf,_len,false);
		
		FlashWritePos+=_len;
		FlashWriteRemaining-= _len;
		
		if(FlashWriteRemaining == 0)
		{
			FlashWriteMode = 0;
			// return positive answer
			strncpy((char*)&fcmcp_responsebuf[0],"---FDONE~~~",11);
			fcmcp_answer(fcmcp_responsebuf,11);
			LED_RED_OFF;
		}
		
		// no further processing!!
	}
	else
	{
		
		// process protocol
		if ( memcmp(_buf,"---FCM~~~",9) == 0)
		{
			// send answer "FCM Version"
			strncpy((char*)&fcmcp_responsebuf[0],"---FCM",6);
			fcmcp_responsebuf[6]= VERSION_MAJOR;
			fcmcp_responsebuf[7]= VERSION_MINOR;
			fcmcp_responsebuf[8]= VERSION_BUILDH;
			fcmcp_responsebuf[9]= VERSION_BUILDL;
			strncpy((char*)&fcmcp_responsebuf[10],"~~~",3);
			fcmcp_answer(fcmcp_responsebuf,9+4); 
			return;
		}
		
		
		if ( memcmp(_buf,"---FER",6) == 0)
		{
			// erase flash
			memcpy(&FlashAdr,&_buf[6],4);
			memcpy(&FlashLen,&_buf[10],4);
			

			static int pageStart, pageEnd;
			pageStart = ((FlashAdr-0x80000000) / (128*4));
			pageEnd = pageStart + ( FlashLen / (128*4));
			
			
			LED_RED_ON;
			for (int i = pageStart;i<=pageEnd;i++)
			{
				if(! flashc_erase_page(i, true)) return; // also check, if empty.
			}
			LED_RED_OFF;
			
			// return positive answer
			strncpy((char*)&fcmcp_responsebuf[0],"---FERSD~~~",11);
			fcmcp_answer(fcmcp_responsebuf,11);
			
			return;
		}
		
		if ( memcmp(_buf,"---FWR",6) == 0)
		{
			// write flash
			memcpy(&FlashAdr,&_buf[6],4);
			memcpy(&FlashLen,&_buf[10],4);
			FlashWritePos = FlashAdr;
			FlashWriteRemaining = FlashLen;
			FlashWriteMode = 1;
			
			// return positive answer
			strncpy((char*)&fcmcp_responsebuf[0],"---FWROK~~~",11);
			fcmcp_answer(fcmcp_responsebuf,11);
			LED_RED_ON;
			
			return;
		}

		if ( memcmp(_buf,"---FRD",6) == 0)
		{
			// read flash
			memcpy(&FlashAdr,&_buf[6],4);
			memcpy(&FlashLen,&_buf[10],4);
			
			for(uint32_t a=FlashAdr;a<FlashAdr+FlashLen;a++)
			{
				while(!fcmcp_IsAnswerPossible())
				{
					// wait some
					delay_ms(11);
				}
				// send byte
				fcmcp_answerByte(*(uint8_t*)a); // yay pointers...
			}
			
			return;
		}
		
		if ( memcmp(_buf,"---SER",6) == 0)
		{
			// get serial number
			
			uint8_t* pser = (uint8_t*)0x80800204; // serial number address ; 120 bit from there = 15 bytes.
			fcmcp_answer(pser,15);
			return;
		}
		
		if ( memcmp(_buf,"---RES",6) == 0)
		{
			// reset CPU
			AVR32_PM.gplp[0] = 0xB007; // out of BL next time
			//StartApplication(); // taugt auch nix, buserror nach Start
			//wdt_reset_mcu(); // geht nicht, wenn wd aus ist!!!
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

	}
}