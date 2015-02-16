/*
   ___ _ _      _   _      ___         _           _   __  __         _      _
  | __| (_)__ _| |_| |_   / __|___ _ _| |_ _ _ ___| | |  \/  |___  __| |_  _| |___
  | _|| | / _` | ' \  _| | (__/ _ \ ' \  _| '_/ _ \ | | |\/| / _ \/ _` | || | / -_)
  |_| |_|_\__, |_||_\__|  \___\___/_||_\__|_| \___/_| |_|  |_\___/\__,_|\_,_|_\___|
    ____  |___/ __  _ _ _    ___     _    _             _  _         _ _ _
   / /\ \  |_  )  \/ | | |  | __|_ _| |__(_)__ _ _ _   | || |_  _ __| (_) |__
  | / _| |  / / () | |_  _| | _/ _` | '_ \ / _` | ' \  | __ | || (_-< | | / /
  | \__| | /___\__/|_| |_|  |_|\__,_|_.__/_\__,_|_||_| |_||_|\_,_/__/_|_|_\_\
   \_\/_/
 
 
  (c) 2012-2015 by Fabian Huslik
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
 
 in linker configuration the flag ,--relax has been removed to prevent the "input not relaxable" error.

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
Important: The -ffunctions-sections in gcc and the -Wl,--gc-sections in linker must be omitted !!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

#include <asf.h>
#include "modules/types.h"
#include "modules/vector.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "gyro.h"
#include "modules/servo_out.h"
#include "servo_in.h"
#include "modules/hottv4.h"
#include "menu/menu.h"
#include "modules/CRC_CCITT.h"
#include "modules/emergency.h"
#include "config.h"
#include "modules/testsuite.h"
#include "TaskControl.h" // for LED fumbling
#include "TaskLED.h" // for LED fumbling

OS_DeclareTask(TaskControl,800);
OS_DeclareTask(TaskTWI,500);
OS_DeclareTask(TaskComm2,800);
OS_DeclareTask(TaskComm0,800);
OS_DeclareTask(TaskLED,500);
OS_DeclareTask(TaskMenu,800);
OS_DeclareTask(TaskNavi,500);

extern void init_pins(void);
extern void init_xtal_pll(void);

#define NO_OS 0

#if NO_OS == 1
	extern void TaskTWI_NO_OS(void);
#endif

int main( void )
{
	OS_DISABLEALLINTERRUPTS; // BL could have been running
	//disable USB ISR !!
	AVR32_USBB.usbcon = 0x03004000; // switch off all interrupts (reset value)
	AVR32_USBB.udcon  = 0x00000100; // detach
	AVR32_USBB.udintclr = 0xffffffff; // disable all ISRs
	
	
	init_xtal_pll();
	init_pins();
	
	// fixme enable BOD
	
	if(AVR32_PM.RCAUSE.por || AVR32_PM.RCAUSE.bod)
	{
		LED_RED_ON;
	}
	else if (AVR32_PM.RCAUSE.wdt)
	{
		LED_RED_ON;
		LED_GREEN_ON;
		LED_BLUE_ON;
		// fixme create fast path to "flight" if this happens!!!
		// meanwhile we crash here
		//emstop(12);
	}
	wdt_opt_t wdopt;
	wdopt.us_timeout_period = 10000000; // 10s // this is the only one parameter.
	wdt_enable(&wdopt); // enable WD
	
	INTC_init_interrupts();
	
	init_crcccitt_tab(); // prepare table for fast CRC calculation

#if NO_OS == 1
	delay_init(BOARD_SYS_HZ);
	OS_ENABLEALLINTERRUPTS;
	while(1)
	{
		TaskTWI_NO_OS(); // debug
		delay_ms(10);
	}

#endif


	#if TEST_RUN == 1
	#warning running unit tests
	test_run();
	
	//while(1); // continue after tests, as failed tests will hang.

	#endif


		
	OS_InitTiming(); 

	// start OS as early as possible and let init the tasks their HW themselves.
	
	#if OS_DO_TESTSUITE == 1
	OS_TestSuite(); // call automated tests of OS. may be removed in production code.
	#endif
	
	OS_CreateTask(TaskControl, OSTSK_CONTROL);
	OS_CreateTask(TaskTWI, OSTSK_TWI);
	OS_CreateTask(TaskComm0, OSTSK_COMM0);
	OS_CreateTask(TaskComm2, OSTSK_COMM2);
	OS_CreateTask(TaskLED, OSTSK_LED);
	OS_CreateTask(TaskMenu, OSTSK_MENU);
	OS_CreateTask(TaskNavi, OSTSK_NAVI);

	OS_CreateAlarm(OSALM_CTRLWAIT,OSTSK_CONTROL);
	OS_CreateAlarm(OSALM_TWIWAIT,OSTSK_TWI);
	OS_CreateAlarm(OSALM_COMM0WAIT,OSTSK_COMM0);
	OS_CreateAlarm(OSALM_COMM2WAIT,OSTSK_COMM2);
	OS_CreateAlarm(OSALM_LEDWAIT,OSTSK_LED);
	OS_CreateAlarm(OSALM_MENUWAIT,OSTSK_MENU);
	OS_CreateAlarm(OSALM_NAVIWAIT,OSTSK_NAVI);
	OS_CreateAlarm(OSALM_TWITIMEOUT,OSTSK_TWI);

	OS_StartExecution() ;
	while(1)
	{
		 // the idle task	
		 
		// you could do phantastic things here...

	}
}	


	


/*
static volatile int32_t rec[100];
static volatile int32_t rcnt;

extern void RecordValue( int32_t v );
//RecordValue(*z);

void RecordValue( int32_t v )
{
	rec[rcnt] = v;
	rcnt++;

	if(rcnt >= 100)
	{
		rcnt = 0;
	}
}
*/


#if OS_USEEXTCHECKS == 1
void OS_ErrorHook(uint8_t ErrNo)
{
	static uint8_t dummy =0;
	
	switch(ErrNo)
	{
		case 2:
			// OS_WaitEvent: waiting in idle is not allowed
			break;	
		case 4:
			// OS_WaitAlarm: waiting in idle is not allowed
			break;
		case 5:
			// OS_MutexGet: invalid Mutex number
			break;
		case 6:
			// OS_MutexRelease: invalid Mutex number
			break;
		case 7:
			// OS_Alarm misconfiguration
			break;
		case 8:
			// OS_WaitAlarm: Alarm was not active
			break;
		case 9:
			// OS_WaitAlarm: Alarm is not assigned to the task (critical!)
			break;
		case 10:
			// Scheduler: Task seems to be waiting inside Mutex
			break;
		case 99:
			// spare
			break;
		default:
			break;	
	}
	
	dummy = ErrNo; // dummy code

	#if OS_DO_TESTSUITE == 1
	asm("breakpoint"); // for automated tests of OS. may be removed in production code.
	#endif
	asm("breakpoint"); // for manual test too...
}
#endif

void OS_ShutdownHook(uint8_t reason)
{
	static uint8_t dummy =0;
	dummy = reason;
	OS_DISABLEALLINTERRUPTS;
	while(1)
	{
		asm("breakpoint");
		wdt_reset_mcu(); // only works, if wd is enabled
	}
}

void OS_CustomISRCode(void)
{
	// reset timer here instead in isr?	
}