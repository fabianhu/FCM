/*
	FabOS for ATMEL AVR32
	
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
*/
#ifndef FABOS_H
#define FABOS_H


// variable types for more tasks

#if OS_NUMTASKS < 32
#define OS_TypeTaskBits_t  uint32_t
#elif OS_NUMTASKS < 64
#define OS_TypeTaskBits_t  uint64_t
#else
	#error "reduce OS_NUMTASKS"
#endif

typedef struct OS_Alarm_tag {
	uint32_t 	TaskID; // Task ID to wake up
	uint32_t 	AlarmTicks; // ticks to count down before reactivation
} OS_Alarm_t;

// *********  the OS data struct
typedef struct FabOS_tag
{
#if OS_USECLOCK == 1
	volatile uint32_t    	OSTicks;					// the OS time, to prevent cluttered results, always use the Function OS_GetTicks() to read it.
#endif
	volatile uint32_t		EventMask[OS_NUMTASKS] ;	// The event masks for all the tasks; Index = Task ID // no event for idle task.
	volatile uint32_t		EventWaiting[OS_NUMTASKS]; // The mask indicates the events, the tasks are waiting for. Index = Task ID

	volatile uint32_t 	MutexOwnedByTask[OS_NUMMUTEX] ;	 // Mutex-owner (contains task ID of owner); only one task can own a mutex.
	volatile uint32_t 	MutexTaskWaiting[OS_NUMTASKS+1] ;	// Mutex-waiters (contains mutex ID) ; Index = Task ID ; The last one is the idle task.

	volatile uint32_t 	CurrTask; 				// here the NUMBER of the actual active task is set.
	volatile OS_TypeTaskBits_t	TaskReadyBits ; 			// here the task activation BITS are set. Task 0 (LSB) has the highest priority.
	volatile uint32_t 	Stacks[OS_NUMTASKS+1];		// actual SP position addresses for the tasks AND the IDLE-task, which uses the ordinary stack! Index = Task ID
	volatile OS_Alarm_t	Alarms[OS_NUMALARMS];  // Holds the number of system clock ticks to wait before the task becomes ready to run.

#if OS_USEMEMCHECKS == 1
	volatile uint32_t*     StackStart[OS_NUMTASKS+1];		// Stack start pointers for checker function
#endif
} FabOS_t;

typedef struct OS_Queue_tag {
	  uint32_t read; // field with oldest content
	  uint32_t write; // always empty field
	  uint32_t chunk;	// size of element
	  uint32_t size;		// number of elements
	  uint8_t* data;	// pointer to data
	} OS_Queue_t;

extern FabOS_t MyOS;

// *********  Macros to simplify the API

#define OS_DeclareQueue(NAME,COUNT,CHUNK) uint8_t OSQD##NAME[(COUNT+1)*CHUNK]; OS_Queue_t NAME = {0,0,CHUNK,(COUNT+1)*CHUNK,OSQD##NAME}

#define OS_DeclareTask(NAME,STACKSIZE) void NAME(void); uint32_t OSStack##NAME[STACKSIZE]

#define OS_CreateTask(NAME, PRIO)  OS_TaskCreateInt(NAME, PRIO, OSStack##NAME , sizeof(OSStack##NAME))

#define OS_CreateAlarm(ALARMID, TASKID) MyOS.Alarms[ALARMID].TaskID = TASKID; MyOS.Alarms[ALARMID].AlarmTicks = 0;

#if OS_TRACE_ON == 1
	#define OS_TRACE(X)\
				OS_Tracebuffer[OS_TraceIdx] = X ; \
				OS_TraceTaskbuffer [OS_TraceIdx++] = MyOS.CurrTask;\
				if(OS_TraceIdx >= sizeof(OS_Tracebuffer)) OS_TraceIdx = 0;
#else
	#define OS_TRACE(X) ;
#endif

// *********  OS function prototypes
void OS_InitTiming (void);

void 	OS_CustomISRCode(void); // do not call; just fill in your code.

void 	OS_StartExecution(void); // Start the operating system

void 	OS_SetEvent(uint8_t TaskID, uint8_t EventMask); // Set one or more events
void	OS_SetEventFromISR(uint8_t TaskID, uint8_t EventMask); // Set one or more events out of a ISR

uint8_t OS_WaitEvent(uint8_t EventMask); //returns event(s) in a mask, which lead to execution

void 	OS_MutexGet(int8_t mutexID); // number of mutexes limited to OS_NUMMUTEX !!! Do not use in ISR !!!!!!
				// Try to get a mutex; execution will block as long the mutex is occupied. If it is free, it is occupied afterwards.

void 	OS_MutexRelease(int8_t mutexID); // release the occupied mutex

void 	OS_SetAlarm(uint8_t AlarmID, uint32_t numTicks ); // set Alarm for the future and continue // set alarm to 0 disable it.

void 	OS_WaitAlarm(uint8_t AlarmID); // Wait for an Alarm set by OS_SetAlarm

uint8_t OS_QueueIn(OS_Queue_t* pQueue , uint8_t *pData); // Put byte into queue, return 1 if q full.

uint8_t OS_QueueOut(OS_Queue_t* pQueue, uint8_t *pData); // Get a byte out of the queue, return 1 if q empty.

void OS_TaskCreateInt( void (*t)(void), uint32_t TaskID, uint32_t *stack, uint32_t stackSize );

#if OS_USEEXTCHECKS == 1
	void OS_ErrorHook(uint8_t);
	void OS_ShutdownHook(uint8_t reason);
#endif

#if OS_USEMEMCHECKS == 1
uint16_t OS_GetUnusedStack (uint8_t TaskID); // give the free stack space for any task as result.
uint8_t OS_GetQueueSpace(OS_Queue_t* pQueue); // give the free space in a queue
#endif

#if OS_USECLOCK == 1
uint32_t 	OS_GetTicks(void); // fills given variable with the OS ticks since start.
#endif

#if OS_DO_TESTSUITE == 1
void 	OS_TestSuite(void); // execute regression test of FabOS (OS development only)
#endif


// Wait for a certain number of OS-ticks (1 = wait to the next timer interrupt)

#if OS_USECOMBINED == 1
uint8_t OS_WaitEventTimeout(uint8_t EventMask, uint8_t AlarmID, uint32_t numTicks ); //returns event on event, 0 on timeout.
#endif

#define OS_WaitTicks(AlarmID,numTicks) do{\
		OS_SetAlarm(AlarmID,numTicks);\
		OS_WaitAlarm(AlarmID);\
		}while(0)



// *********  CPU related assembler stuff

#define OS_DISABLEALLINTERRUPTS asm("ssrf 16");asm("nop");asm("nop"); // flush the pipeline
#define OS_ENABLEALLINTERRUPTS asm("csrf 16");

#define OS_ALLOWSCHEDULING 		asm("csrf 17");	// turn Timer Interrupt ON on Level 3! (todo maybe switch isr directly on/off)
#define OS_PREVENTSCHEDULING 	asm("ssrf 17");asm("nop");asm("nop"); // turn Timer Interrupt OFF

// Task switching context stack layout:
// - = not on stack
// * = automatically on stack by ISR - manually in normal call
// + = stored "manually"

// - R15 = Program Counter (never stored except in return addresses)
// - R13 = Stack Pointer (not stored on stack, as this would be quite useless.)

// ******** stack ends here!
// + R7 // the SP points here before task switch.
// + R6
// + R5
// + R4
// + R3
// + R2
// + R1
// + R0
// * Status register just before ISR (ok)
// * R15 = (PC) = return address (ok)
// * R14 = Link Register (ok)
// * R12 = parameter & return value
// * R11
// * R10
// * R9
// * R8
// ******** stack starts here and goes upwards! This line is a illegal address for this task.

// save the registers, as a interrupt would do: // NEVER use inside of ISR!
#define OS_int_Push_ISR_PseudoFrameSCALL()\
asm volatile(\
	"pushm	r10-r12			\n\t" /*push*/\
	"pushm	LR				\n\t" /*LR*/\
	"ld.w	r10,SP[+5*4]	\n\t" /*relocate SR on stack*/\
	"st.w	--SP,r10		\n\t" /*relocate SR on stack*/\
	"ld.w	r10,SP[+5*4]	\n\t" /*relocate PC on stack*/\
	"st.w	--SP,r10		\n\t" /*relocate PC on stack*/\
	"st.w	SP[+6*4],r9		\n\t" /*push to final location*/\
	"st.w	SP[+7*4],r8		\n\t" /*push to final location*/\
:\
:[SR] "i" (AVR32_SR)\
);

 // restore the registers, as a interrupt would do:
#define OS_int_Pop_ISR_PseudoFrameRETS()\
asm volatile(\
	"ld.w	r9,SP[+6*4]		\n\t" /*load R8 and R9 from stack, so that the space is freed.*/\
	"ld.w	r8,SP[+7*4]		\n\t" /*store the future SR and PC onto lower bound of stack (address higher!)*/\
	"popm	r10				\n\t" /*SR to lowest stack location*/\
	"st.w   SP[+5*4],r10	\n\t" \
	"popm	r10				\n\t" /*PC to lowest stack location*/\
	"st.w   SP[+5*4],r10	\n\t" \
	"popm	LR				\n\t" /*pop the rest*/\
	"popm	r10-r12			\n\t" /*pop the rest*/\
	"rets \n\t"	/*restore the SREG and PC by CPU*/\
    :\
    :[SR] "i" (AVR32_SR)\
  );
 
// save the registers, the interrupt does not take care of:
#define OS_int_Push_Std_Frame()\
asm volatile(\
	/*the rest*/\
	"pushm	r0-r7	\n\t"\
    :\
    :\
  );	

// restore the registers, the interrupt does not take care of:
#define OS_int_Pop_Std_Frame()\
asm volatile(\
	/*the rest*/\
	"popm	r0-r7	\n\t"\
    :\
    :\
  );

 
// *********  some final warning calculations

#if (!defined(OS_NUMTASKS		))||\
	(!defined(OS_NUMMUTEX 		))||\
	(!defined(OS_NUMALARMS		))||\
	(!defined(OS_USECLOCK 		))||\
	(!defined(OS_USECOMBINED 	))||\
	(!defined(OS_USEEXTCHECKS   ))||\
	(!defined(OS_USEMEMCHECKS 	))||\
	(!defined(OS_UNUSEDMASK 	))||\
	(!defined(OS_TRACE_ON       ))||\
	(!defined(OS_TRACESIZE      ))
	#error not all defines in FabOS_config.h are done as described here (FabOS.h) below!
#endif


/* Example defines for FabOS_config.h     

#define OS_NUMTASKS  4 // Number of (OS_Create)Tasks ; never >8 (idle task is not counted here!)
#define OS_NUMMUTEX  3 // Number of Mutexes
#define OS_NUMALARMS 5 // Number of Alarms

#define OS_ScheduleISR TIMER1_COMPA_vect // Interrupt Vector used for OS-tick generation (check out CustomOS_ISRCode if you want to add isr code)

#define OS_USECLOCK 1 		// Use "OS_GetTicks()" which returns a 32bit timer tick
#define OS_USECOMBINED 1 	// Use "OS_WaitEventTimeout()" which is easier to use, than combining alarms and events to get the functionality.
#define OS_USEEXTCHECKS 1	// check wrong usage of OS API -> does not work, but no damage to OS stability.
#define OS_USEMEMCHECKS 1 	// Enable "OS_get_unused_Stack()" and "OS_GetQueueSpace()"
#define OS_UNUSEDMASK 0xEE  // unused Stack RAM will be filled with this byte, if OS_USEMEMCHECKS == 1.
#define OS_TRACE_ON  1 		// enable trace to OS_Tracebuffer[]
#define OS_TRACESIZE 1000	// size of OS_Tracebuffer[] (depending on memory left ;-)

*/


#if OS_NUMMUTEX > OS_NUMTASKS 
	#warning "more mutexes than tasks? are you serious with that concept?"
#endif

#if (OS_DO_TESTSUITE == 1) && (\
		(	OS_NUMTASKS 	!=5	) ||\
		(	OS_NUMMUTEX 	!=3	) ||\
		(   OS_NUMALARMS    !=6 ) ||\
		(	OS_USEEXTCHECKS !=1 ) ||\
		(	OS_USECLOCK 	!=1	) ||\
		(	OS_USEMEMCHECKS !=1	) ||\
		(	OS_USECOMBINED 	!=1	) \
		) 
		#error "please configure the defines for the testsuite as stated above!"
#endif

#if OS_USEMEMCHECKS == 0
	#undef UNUSEDMASK
#endif


#endif // FABOS_H
