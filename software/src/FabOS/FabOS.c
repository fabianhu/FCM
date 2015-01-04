/*
	FabOS for ATMEL AVR
	
	(c) 2008-2015 Fabian Huslik
    
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

#include "asf.h"
#include "../FabOS_config.h"
#include "FabOS.h"
#include "timing.h"

FabOS_t MyOS; // the global instance of the OS struct

// internal prototypes

__attribute__ ((naked)) void OS_Reschedule_SCALL(void); // internal: Reschedule 
void OS_Reschedule(void); // internal: Trigger re-scheduling out of normal program flow using SCALL instruction

static int8_t OS_GetNextTaskNumber(void); // internal: get the next task to be run// which is the next task (ready and highest (= rightmost); prio);?

static void OS_Int_ProcessAlarms(void);


#if OS_TRACE_ON == 1
	uint8_t OS_Tracebuffer[OS_TRACESIZE];
	uint8_t OS_TraceTaskbuffer[OS_TRACESIZE];
	uint32_t dumy1;
	uint32_t OS_TraceIdx;
	uint32_t dumy2;
#endif


// From linker script
extern uint32_t __heap_start__;


/*
 __attribute__((__always_inline__)) uint32_t OS_GetSP(void);
 __attribute__((__always_inline__)) void OS_PutSP(uint32_t adr);*/


static __attribute__((__always_inline__)) uint32_t OS_GetSP(void) 
{
	register uint32_t res asm ("r12");
	asm ("mov %[res],SP":[res]"=r"(res));
	return res;
}

static __attribute__((__always_inline__)) void OS_PutSP(uint32_t adr)
{
	asm ("mov SP,%[adr]"::[adr]"r"(adr));
}

//__attribute__((__always_inline__)) uint32_t OS_GetSREG(void);
/*static __attribute__((__always_inline__)) uint32_t OS_GetSREG(void)
{
	register uint32_t res asm ("r12");
	asm ("mfsr %[res],%[SR]":[res]"=r"(res):[SR] "i" (AVR32_SR));
	return res;
}*/



// *********  Timer Interrupt
// The naked attribute tells the compiler not to add code to push the registers it uses onto the stack or even add a RETI instruction at the end. 
// It just compiles the code inside the braces.
// *** No direct use of stack space inside a naked function, except embedding it into a function, as this creates a valid stack frame.
// the ISR hardware pushes R8 to R12 onto the stack, so it should be safe to use these directly.
__attribute__((__interrupt__, __naked__))
static void isr_cyclic1ms(void)
{
	// this is the cyclic timer interrupt. 
	OS_int_Push_Std_Frame(); // this should be the first action we do!!!
	// the rest is saved in isr frame by hardware.
	
	MyOS.Stacks[MyOS.CurrTask] = OS_GetSP(); // catch the SP before we (possibly) do anything with it.

	AVR32_TC.channel[TIMERCHANNEL_OS].sr;	// read clears the bits 	

	OS_TRACE(2);
	OS_CustomISRCode(); // Caller of Custom ISR code to be executed inside this ISR on the last active tasks stack
	OS_TRACE(3);

#if OS_USECLOCK == 1
	MyOS.OSTicks++; 	// tick the RT-clock...
#endif
	
	OS_Int_ProcessAlarms(); // Calculate alarms; function uses stack
	OS_TRACE(4);

	// task is to be run
	MyOS.CurrTask = OS_GetNextTaskNumber() ;

	OS_TRACE(5);

	OS_PutSP(MyOS.Stacks[MyOS.CurrTask]); // restore stack pointer

	OS_int_Pop_Std_Frame();
	asm volatile("rete");  // return from this isr. (naked)
}



void OS_InitTiming (void)
{
	// OS timer ISR is on level 0. NO other ISR is allowed on this level.
	// The mutex between inline schedulung and timer based schedulung is assured with this level.
	// If some other ISR is doing special scheduling, the Level must also be 0 !!!
	
	
	// Timer 1:
	// Counter for 1ms interrupt
	AVR32_TC.channel[TIMERCHANNEL_OS].CMR.capture.tcclks = AVR32_TC_TCCLKS_TIMER_CLOCK2; // = PBA/2
	AVR32_TC.channel[TIMERCHANNEL_OS].CMR.capture.cpctrg = 1; // enable compare match on reg. C and reset the Counter.
	AVR32_TC.channel[TIMERCHANNEL_OS].IER.cpcs = 1; // enable interrupt 1 = yes, on compare match on reg. C
	AVR32_TC.channel[TIMERCHANNEL_OS].RC.rc = BOARD_SYS_HZ/2000; //  compare match on reg. C after 1 ms (/2 because of PBA/2 and /1000 because of ms
	//unten ! AVR32_TC.channel[TIMERCHANNEL_OS].CCR.clken = 1; // enable clock
	
	INTC_register_interrupt(&isr_cyclic1ms,AVR32_TC_IRQ1,AVR32_INTC_INTLEVEL_INT0); // for FabOS // this is the only ISR allowed on this level!
	AVR32_TC.channel[TIMERCHANNEL_OS].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;	
}


// *********  Internal scheduling and priority stuff

static void OS_Int_ProcessAlarms(void)
{
	uint8_t alarmID;
	OS_TRACE(7);

	// handling of OS_Wait / Alarms
	for(alarmID=0; alarmID < OS_NUMALARMS; alarmID++ )
	{ 
		if( MyOS.Alarms[alarmID].AlarmTicks > 0 ) // this task has to wait
		{
			OS_TRACE(8);
			MyOS.Alarms[alarmID].AlarmTicks--;
			if( MyOS.Alarms[alarmID].AlarmTicks == 0 ) // if now task is ready, it will be activated.
			{
				OS_TRACE(9);
				MyOS.TaskReadyBits |= 1<<(MyOS.Alarms[alarmID].TaskID) ; // now it is finished with waiting
			}
			else
			{	
				OS_TRACE(10);
			}
			
		}
		else
		{
			OS_TRACE(11);
		}
	}
}

void OS_Reschedule(void)
{
	OS_PREVENTSCHEDULING;
	asm("scall");
	OS_ALLOWSCHEDULING;
}


// Supervisor Call handler
// Attention: only to be used AFTER OS_PREVENTSCHEDULING and only OUTSIDE of any ISR
__attribute__ ((naked)) 
void OS_Reschedule_SCALL(void) //with "__attribute__ ((naked))" because it puts R7 and PC on stack otherwise!
{
	OS_int_Push_ISR_PseudoFrameSCALL(); 
	OS_int_Push_Std_Frame();
	MyOS.Stacks[MyOS.CurrTask] = OS_GetSP(); // catch the SP before we (possibly) do anything with it.
	
	OS_TRACE(13);

	// task is to be run
	MyOS.CurrTask = OS_GetNextTaskNumber();

	OS_TRACE(14);

	OS_PutSP(MyOS.Stacks[MyOS.CurrTask]); // restore stack pointer// set Stack pointer
	OS_int_Pop_Std_Frame();
	OS_int_Pop_ISR_PseudoFrameRETS(); // soo, that's it, this macro does not return. 
}


static int8_t OS_GetNextTaskNumber() // which is the next task (ready and highest (= rightmost) prio)?
{
	uint8_t Task;
	uint8_t	next= OS_NUMTASKS; // NO task is ready, which one to execute?? the idle task !!;

	OS_TypeTaskBits_t ReadyMask = MyOS.TaskReadyBits; // make working copy
	
	OS_TRACE(20);

	for (Task=0;Task<OS_NUMTASKS;Task++)
	{
		if (ReadyMask & 0x01) // last bit set
		{	
			OS_TRACE(21);
			next =  Task; // this task is the one to be executed
			break;
		}
		else
		{
			OS_TRACE(22);
			ReadyMask= (ReadyMask>>1); // shift to right; "Task" and the shift count is synchronous.
		}
	}
	// now "next" is the next highest prio task.

	// look in mutex waiting list, if any task is blocking "next", then "next" must not run.
	// if next is waiting for a task and the mutex is owned by a task then give the waiting the run.
	if (MyOS.MutexTaskWaiting[next] != 0xff) // "next" is waiting for a mutex
	{
		OS_TRACE(23);
		if(MyOS.MutexOwnedByTask[MyOS.MutexTaskWaiting[next]]!=0xff) // is the mutex still owned?
		{
			OS_TRACE(24);
			// which task is blocking it?
			next = MyOS.MutexOwnedByTask[MyOS.MutexTaskWaiting[next]]; 
			// the blocker gets the run.
			// this is also a priority inversion.
			if(((1<<next)&MyOS.TaskReadyBits) == 0)  // special case, where the blocker is not ready to run (somehow illegal waiting inside mutex)
			{
				OS_TRACE(25);
				
				#if OS_USEEXTCHECKS == 1
				
				OS_ErrorHook(10);// seems to be waiting inside Mutex!
				
				#endif
				
				next = OS_NUMTASKS; // the idle task gets the run...
			}
			else
			{
				OS_TRACE(26);
			}
		}
	}
	else
	{
			OS_TRACE(27);
	}
	return next;
}


// internal task create function
// stacksize in Bytes!
void OS_TaskCreateInt( void(*t)(void), uint32_t TaskID, uint32_t *stack, uint32_t stackSize )
{
	uint32_t z ;
	OS_TRACE(28);

#if OS_USEMEMCHECKS == 1
	// "colorize" the stacks
	for (z=0;z<stackSize/4;z++)
	{
		stack[z] = (OS_UNUSEDMASK)|TaskID;
	}

	MyOS.StackStart[TaskID]= stack;
	
	// check alignment
	if(((uint32_t)stack) & 0x3) // last 2 bits set?
	{
		OS_ShutdownHook(1);
	}
	if((stackSize) & 0x3) // last 2 bits set?
	{
		OS_ShutdownHook(2);
	}
	
#endif

#define STACKFRAMEWORDSIZE 16

	MyOS.TaskReadyBits |= 1<<TaskID ;  // indicate that the task exists and is ready to run.
	
	for (z=(stackSize/4)-STACKFRAMEWORDSIZE;z<stackSize/4;z++) // clear the stack frame space (should already be, but you never know...)
	{
		stack[z] = 0;
	}
		
	MyOS.Stacks[TaskID] = (uint32_t)stack + stackSize; // Point the task's SP after the end address of the array that represents its stack.
	MyOS.Stacks[TaskID] -= STACKFRAMEWORDSIZE*4;   // Create a stack frame according the layout. SP points to the first in the frame.

	*(uint32_t*)(MyOS.Stacks[TaskID]+9*4) = (uint32_t)(t); // put the address of the task-function inside the prepared stack frame
	*(uint32_t*)(MyOS.Stacks[TaskID]+8*4) = 0x00400000; // Provide the SR with normal operation mode

}

//  testing only
#define clutterRegsIdle()\
asm volatile(\
	"mov	r0,0x1100 \n\t"\
	"mov	r1,0x1101 \n\t"\
	"mov	r2,0x1102 \n\t"\
	"mov	r3,0x1103 \n\t"\
	"mov	r4,0x1104 \n\t"\
	"mov	r5,0x1105 \n\t"\
	"mov	r6,0x1106 \n\t"\
	"mov	r7,0x1107 \n\t"\
	"mov	r8,0x1108 \n\t"\
	"mov	r9,0x1109 \n\t"\
	"mov	r10,0x110a \n\t"\
	"mov	r11,0x110b \n\t"\
	"mov	r12,0x110c \n\t"\
    :\
    :\
  );


// Start the operating system
void OS_StartExecution()
{
	uint32_t i;
	OS_TRACE(29);
	for(i=0; i < OS_NUMTASKS+1; i++ ) // init mutexes
	{ 
		MyOS.MutexTaskWaiting[i] = 0xff;
	}
	for(i=0; i < OS_NUMMUTEX; i++ ) 
	{ 
		MyOS.MutexOwnedByTask[i] = 0xff;
	}

/*#if OS_USEMEMCHECKS == 1
	uint32_t* stack = (uint32_t*)OS_GetSP();
	MyOS.StackStart[OS_NUMTASKS] = (uint32_t*)&__heap_start__;
	// "colorize" the idle stack
	while (stack > MyOS.StackStart[OS_NUMTASKS])
	{
		*--stack = (OS_UNUSEDMASK) | OS_NUMTASKS;
	}
#endif  todo getstacks of idle task will not work.*/ 

#if OS_USECLOCK == 1
	MyOS.OSTicks = 0L; 	// reset the RT-clock...
#endif

	//store THIS context for idling!!
	MyOS.CurrTask = OS_NUMTASKS;
	
	OS_ENABLEALLINTERRUPTS; // global enable interrupts	(also starts scheduling ISR)
	OS_Reschedule(); // Here every task is executed at least once. (Scheduling is enabled afterwards)
	
}


// ************************** MUTEX

// Try to get a mutex; execution will block as long the mutex is occupied. If it is free, it is occupied afterwards.
// Never within ISR!
void OS_MutexGet(int8_t mutexID)
{
#if OS_USEEXTCHECKS == 1
	if(mutexID >= OS_NUMMUTEX)
	{
		OS_ErrorHook(5);// OS_MutexGet: invalid Mutex number
		return;
	}
#endif
	OS_PREVENTSCHEDULING;
	OS_TRACE(31);
	if( MyOS.MutexOwnedByTask[mutexID] != 0xff) // as long as anyone else is the owner..
	{
		OS_TRACE(32);
		MyOS.MutexTaskWaiting[MyOS.CurrTask] = mutexID; // set waiting info for priority inversion of scheduler
		OS_Reschedule(); // also re-enables interrupts...
		OS_PREVENTSCHEDULING;
		OS_TRACE(33);
		// we only get here, if the other has released the mutex.
		MyOS.MutexTaskWaiting[MyOS.CurrTask] = 0xff; // remove waiting info
	}
	OS_TRACE(34);
	MyOS.MutexOwnedByTask[mutexID] = MyOS.CurrTask; // tell others, that I am the owner.
	OS_ALLOWSCHEDULING;
}

// release the occupied mutex
// Never within ISR!
void OS_MutexRelease(int8_t mutexID)
{
#if OS_USEEXTCHECKS == 1
	if(mutexID >= OS_NUMMUTEX)
	{
		OS_ErrorHook(6);// OS_MutexRelease: invalid Mutex number
		return;
	}
#endif
	OS_PREVENTSCHEDULING;
	OS_TRACE(35);
	MyOS.MutexOwnedByTask[mutexID] = 0xff; // tell others, that no one is the owner.
	OS_Reschedule() ; // re-schedule; will wake up waiting task, if higher prio.
}

// ************************** EVENTS

// do not use in ISR!
void OS_SetEvent(uint8_t TaskID, uint8_t EventMask) // Set one or more events
{
	#if OS_USEEXTCHECKS == 1
	if(EventMask == 0)
	{
		OS_ErrorHook(12);// OS_SetEvent: event mask empty!
		return;
	}
	#endif
	
	OS_PREVENTSCHEDULING;
	OS_TRACE(36);
	MyOS.EventMask[TaskID] |= EventMask; // set the event mask, as there may be more events than waited for.

	if(EventMask & MyOS.EventWaiting[TaskID]) // Targeted task is waiting for one of this events
	{
		OS_TRACE(37);
		// wake up this task directly
		MyOS.TaskReadyBits |= 1<<TaskID ;   // Make the task ready to run again.

		// the waked up task will then clean up all entries to the event

		OS_Reschedule() ; // re-schedule; will wake up the sleeper directly, if higher prio.
	}
	else
	{
		OS_TRACE(38);
		// remember the event and task continues on its call of WaitEvent directly. 
		OS_ALLOWSCHEDULING;
	}
}

// call out of ISR, no reschedule is done, the Event is delayed to the next Reschedule.
void OS_SetEventFromISR(uint8_t TaskID, uint8_t EventMask) // Set one or more events out of a ISR
{
	#if OS_USEEXTCHECKS == 1
	if(EventMask == 0)
	{
		OS_ErrorHook(12);// OS_SetEventFromISR: event mask empty!
		return;
	}
	#endif
	// OS_PREVENTSCHEDULING already off in any ISR, as OS-tick is the lowest prio. ;
	OS_TRACE(39);
	MyOS.EventMask[TaskID] |= EventMask; // set the event mask, as there may be more events than waited for.

	if(EventMask & MyOS.EventWaiting[TaskID]) // Targeted task is waiting for one of this events
	{
		OS_TRACE(40);
		// wake up this task directly
		MyOS.TaskReadyBits |= 1<<TaskID ;   // Make the task ready to run again.

		// the waked up task will then clean up all entries to the event

		// not in ISR !!! OS_Reschedule() ; // re-schedule; will wake up the sleeper directly, if higher prio.
	}
	else
	{
		OS_TRACE(41);
		// remember the event and task continues on its call of WaitEvent directly.
		//  not in ISR !!! OS_ALLOWSCHEDULING;
	}
}


// never in ISR!
uint8_t OS_WaitEvent(uint8_t EventMask) //returns event(s), which lead to execution
{
#if OS_USEEXTCHECKS == 1
	if(MyOS.CurrTask == OS_NUMTASKS) 
	{
		OS_ErrorHook(2);// OS_WaitEvent: waiting in idle is not allowed
		return 0; 
	}
	if(EventMask == 0)
	{
		OS_ErrorHook(12);// OS_SetEvent: event mask empty!
		return 0;
	}
#endif

	uint8_t ret;
	OS_PREVENTSCHEDULING;
	OS_TRACE(42);
	
	if(!(EventMask & MyOS.EventMask[MyOS.CurrTask])) // This task is Not having one of these events active
	{
		OS_TRACE(43);
		MyOS.EventWaiting[MyOS.CurrTask] = EventMask; // remember what this task is waiting for
		// no event yet... waiting
		MyOS.TaskReadyBits &= ~(1<<MyOS.CurrTask) ;     // indicate that this task is not ready to run.

		OS_Reschedule() ; // re-schedule; will be waked up here by "SetEvent" or alarm
		OS_PREVENTSCHEDULING;
		OS_TRACE(44);

		MyOS.EventWaiting[MyOS.CurrTask] = 0; // no more waiting!
	}
	ret = MyOS.EventMask[MyOS.CurrTask] & EventMask;
	// clear the events:
	MyOS.EventMask[MyOS.CurrTask] &= ~EventMask; // the actual events minus the ones, which have been waited for 
	OS_TRACE(45);
	OS_ALLOWSCHEDULING;
	return ret;
}


// ************************** ALARMS


void OS_SetAlarm(uint8_t AlarmID, uint32_t numTicks ) // set Alarm for the future and continue // set alarm to 0 disable an alarm.
{
#if OS_USEEXTCHECKS == 1
	if(AlarmID >= OS_NUMALARMS)// check for ID out of range
	{
		OS_ErrorHook(7); // ID bigger than array size
		return;
	}
#endif	
	OS_PREVENTSCHEDULING;
	OS_TRACE(46);
	MyOS.Alarms[AlarmID].AlarmTicks = numTicks ;
	OS_ALLOWSCHEDULING;
}

void OS_WaitAlarm(uint8_t AlarmID) // Wait for any Alarm set by OS_SetAlarm
{
#if OS_USEEXTCHECKS == 1
	if(AlarmID >= OS_NUMALARMS)// check for ID out of range
	{
		OS_ErrorHook(7); // ID bigger than array size
		return;
	}
	if(MyOS.CurrTask == OS_NUMTASKS) 
	{
		OS_ErrorHook(4);// OS_WaitAlarm: waiting in idle is not allowed
		return;  
	}
	if(MyOS.Alarms[AlarmID].TaskID != MyOS.CurrTask) // Alarm is not assigned!
	{
		OS_TRACE(47);
		OS_ErrorHook(9); // OS_WaitAlarm: Alarm is not assigned to the task
		return;  
	}
#endif

	OS_PREVENTSCHEDULING; // re-enabled by OS_Schedule()
	OS_TRACE(48);
	if(MyOS.Alarms[AlarmID].AlarmTicks == 0 ) // notice: this "if" could be possibly omitted.
	{
#if OS_USEEXTCHECKS == 1 && 0
		OS_ErrorHook(8); // OS_WaitAlarm: Alarm was not active
#endif
		OS_ALLOWSCHEDULING; // just continue
	}
	else
	{
		OS_TRACE(49);
		MyOS.TaskReadyBits &= ~(1<<MyOS.CurrTask) ;  // Disable this task
		OS_Reschedule();  // re-schedule; let the others run...	
	}
}

// ************************** QUEUES
/* in FabOS.h we have:
typedef struct OS_Queue_tag {
	  uint8_t read; // field with oldest content
	  uint8_t write; // always empty field
	  uint8_t chunk;
	  uint8_t size;
	  uint8_t* data;
	} OS_Queue_t;
*/

// todo neues Konzept ??
/*
	1. void* Queue_getPush()
	2. write to ptr, if not NULL
	3. void* Queue_peek()
	4. read value, if not NULL


*/


uint8_t OS_QueueIn(OS_Queue_t* pQueue , uint8_t* pByte)
{
	uint8_t i;
	OS_PREVENTSCHEDULING;
	OS_TRACE(50);
	if (pQueue->write + pQueue->chunk == pQueue->read || (pQueue->read == 0 && pQueue->write + pQueue->chunk == pQueue->size))
	{
		OS_TRACE(51);
		OS_ALLOWSCHEDULING;
		return 1;  // queue full
	}

	for(i=0;i<pQueue->chunk;i++)
	{
		pQueue->data[pQueue->write] = *pByte++;
		pQueue->write = pQueue->write + 1;
		if (pQueue->write >= pQueue->size)
			pQueue->write = 0;
	}
	OS_TRACE(52);
	OS_ALLOWSCHEDULING;
	return 0;
}
 
uint8_t OS_QueueOut(OS_Queue_t* pQueue , uint8_t* pByte)
{
	uint8_t i;
	OS_PREVENTSCHEDULING;
	OS_TRACE(53);
	if (pQueue->read == pQueue->write)
	{
		OS_TRACE(54);
		OS_ALLOWSCHEDULING;
		return 1; // queue empty
	}

	for(i=0;i<pQueue->chunk;i++)
	{
		*pByte++ = pQueue->data[pQueue->read];
		pQueue->read = pQueue->read + 1;
		if (pQueue->read >= pQueue->size)
			pQueue->read = 0;
	}
	OS_TRACE(55);
	OS_ALLOWSCHEDULING;
	return 0;
}

#if OS_USEMEMCHECKS == 1
uint8_t OS_GetQueueSpace(OS_Queue_t* pQueue)
{
	OS_TRACE(56);
	if (pQueue->read < pQueue->write)
		return pQueue->size - pQueue->write + pQueue->read;
	else if(pQueue->read > pQueue->write)
		return  (pQueue->read - pQueue->write)-1;
	return pQueue->size-1;
}
#endif

// *********************** Aux functions

#if OS_USEMEMCHECKS == 1
// give the free stack space for any task in function result.
uint16_t OS_GetUnusedStack (uint8_t TaskID)
{
   //OS_TRACE(57);
   uint32_t unused = 0;
   volatile uint32_t* p = MyOS.StackStart[TaskID]; 

   do
   {
      if (*p++ != (OS_UNUSEDMASK | TaskID))
         break;

      unused++;
   } while (p <= (uint32_t*) MyOS.Stacks[TaskID]);

      return unused;
}
#endif

#if OS_USECLOCK == 1
// fills given variable with the OS ticks since start.
uint32_t OS_GetTicks(void) 
{
	/*32 bit are task safe in UC3
	  uint32_t ret;
	OS_PREVENTSCHEDULING;
		OS_TRACE(58);
		ret = MyOS.OSTicks;
	OS_ALLOWSCHEDULING;
	return ret;*/
	return MyOS.OSTicks;
}
#endif


#if OS_USECOMBINED
uint8_t OS_WaitEventTimeout(uint8_t EventMask, uint8_t AlarmID, uint32_t numTicks ) //returns event on event, 0 on timeout.
{
	uint8_t ret;
	OS_TRACE(59);
	OS_SetAlarm(AlarmID,numTicks); // set timeout
	ret = OS_WaitEvent(EventMask);
	if(ret & EventMask)
	{
		// event occured
		OS_TRACE(60);
		OS_SetAlarm(AlarmID,0); // disable timeout
		return ret;
	}
	else
	{
		OS_TRACE(61);
		// timeout occured
		return ret;
	}
}
#endif
