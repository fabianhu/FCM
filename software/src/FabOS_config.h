/*
	FabOS for ATMEL AVR32 user configuration file
	
	(c) 2008-2014 Fabian Huslik
    
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

	Please change this file to your needs.
*/

// *********  USER Configurable Block BEGIN
#define OS_DO_TESTSUITE 0	// compile and execute the automated software tests. Set to 0 for production use of OS.

#if OS_DO_TESTSUITE == 0

#define OS_NUMTASKS  7 // Number of (OS_Create)Tasks ; never >64 (idle task is not counted here!)
#define OS_NUMMUTEX  2 // Number of Mutexes
#define OS_NUMALARMS 8 // Number of Alarms // todo maybe count

#define OS_USECLOCK 1 		// Use "OS_GetTicks()" which returns a 32bit timer tick
#define OS_USECOMBINED 1 	// Use "OS_WaitEventTimeout()" which is easier to use, than combining alarms and events to get the functionality.
#define OS_USEEXTCHECKS 1	// check wrong usage of OS API -> does not work, but no damage to OS stability.
#define OS_USEMEMCHECKS 1 	// Enable "OS_get_unused_Stack()" and "OS_GetQueueSpace()"
#define OS_UNUSEDMASK 0xEEEEEE00  // unused Stack RAM will be filled with this byte, if OS_USEMEMCHECKS == 1.
#define OS_TRACE_ON  0 		// enable trace to OS_Tracebuffer[]
#define OS_TRACESIZE 1000	// size of OS_Tracebuffer[] (depending on memory left ;-)

#endif

// Task definitions = priorities, 0 = highest
#define OSTSK_TWI 0
#define OSTSK_COMM0 1	// high prio for HoTT (Exact timing)
#define OSTSK_CONTROL 2
#define OSTSK_COMM2 3	// GPS
#define OSTSK_LED 4
#define OSTSK_NAVI 5
#define OSTSK_MENU 6

// Event Names (Event is a bit in a bitfield !!!!)
#define OSEVT_CTLWAKEUP  (1<<0)
#define OSEVT_GPSREADY   (1<<1)
#define OSEVT_MENUWAKEUP (1<<2)
#define OSEVT_MAVRX		 (1<<3)
#define OSEVT_TWIRDY     (1<<4)

// Mutex Names
#define OSMTX_COMM 0
#define OSMTX_TWI 1

// Alarm Names
#define OSALM_CTRLWAIT	0
#define OSALM_TWIWAIT	1
#define OSALM_COMM0WAIT	2
#define OSALM_COMM2WAIT	3
#define OSALM_LEDWAIT	4
#define OSALM_MENUWAIT	5
#define OSALM_NAVIWAIT	6
#define OSALM_TWITIMEOUT 7



// *********  USER Configurable Block END 


#if OS_DO_TESTSUITE == 1

#define OS_NUMTASKS  5 // Number of (OS_Create)Tasks ; never >64 (idle task is not counted here!)
#define OS_NUMMUTEX  3 // Number of Mutexes
#define OS_NUMALARMS 6 // Number of Alarms

#define OS_USECLOCK 1 		// Use "OS_GetTicks()" which returns a 32bit timer tick
#define OS_USECOMBINED 1 	// Use "OS_WaitEventTimeout()" which is easier to use, than combining alarms and events to get the functionality.
#define OS_USEEXTCHECKS 1	// check wrong usage of OS API -> does not work, but no damage to OS stability.
#define OS_USEMEMCHECKS 1 	// Enable "OS_get_unused_Stack()" and "OS_GetQueueSpace()"
#define OS_UNUSEDMASK 0xDEAD0000  // unused Stack RAM will be filled with this byte, if OS_USEMEMCHECKS == 1.
#define OS_TRACE_ON  1 		// enable trace to OS_Tracebuffer[]
#define OS_TRACESIZE 1000	// size of OS_Tracebuffer[] (depending on memory left ;-)

#endif
