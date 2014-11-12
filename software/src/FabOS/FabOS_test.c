/*
	FabOS for ATMEL AVR32 Test suite

    (c) 2008-2014 Fabian Huslik

	This file implememts automated software testing of the OS modules.
	This file should only be altered if really necessary.
	It does not contain any code, which is useful for production use of FabOS.

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

remember:

avr-gcc -mmcu=atmega32 -E FabOS_test.c >bla.c  invoke preprocessor only
*/

#include "asf.h"
#include "../FabOS_config.h"
#include "FabOS.h"

#if OS_DO_TESTSUITE == 1 // compile only if tests are activated (OS development use only)

extern void OS_Reschedule(void);

#define clutterRegs()\
asm volatile(\
	"mov	r0,0xaa00 \n\t"\
	"mov	r1,0xaa01 \n\t"\
	"mov	r2,0xaa02 \n\t"\
	"mov	r3,0xaa03 \n\t"\
	"mov	r4,0xaa04 \n\t"\
	"mov	r5,0xaa05 \n\t"\
	"mov	r6,0xaa06 \n\t"\
	"mov	r7,0xaa07 \n\t"\
	"mov	r8,0xaa08 \n\t"\
	"mov	r9,0xaa09 \n\t"\
	"mov	r10,0xaa0a \n\t"\
	"mov	r11,0xaa0b \n\t"\
	"mov	r12,0xaa0c \n\t"\
    :\
    :\
  );	


OS_DeclareTask(TestTask0,200);
OS_DeclareTask(TestTask1,200);
OS_DeclareTask(TestTask2,200);
OS_DeclareTask(TestTaskhi,100);
OS_DeclareTask(TestTasklo,100);

volatile uint16_t r,s,t,u;
volatile uint8_t testvar=0;

OS_DeclareQueue(TestQ,64,1);
OS_DeclareQueue(TestQLong,10,4);

//OS_Queue_t TestQ ={0,0,{}};

uint32_t longQtest[10] = {0x11223344, 0x22334455, 0x33445566, 0x44556677, 0x55667788, 0x66778899, 0x778899aa, 0x8899aabb, 0x99aabbcc, 0xaabbccdd};

#define MAXTESTCASES 12 // one more than the max "case" ID.
uint8_t testcase = 0; // test case number
uint8_t TestResults[MAXTESTCASES]; // test result array (0= OK)
uint8_t TestProcessed[MAXTESTCASES]; // test passed array (number of processed assertions

#define assert(X) do{								\
	OS_DISABLEALLINTERRUPTS;	\
	if(!(X))					\
	{							\
		TestResults[testcase]++;\
		asm("breakpoint");	\
	}							\
	TestProcessed[testcase]++;	\
	OS_ENABLEALLINTERRUPTS;		\
}while(0)


#define OSALMTest0 0
#define OSALMTest1 1
#define OSALMTest2 2
#define OSALMTest3 3
#define OSALMTesthi 4
#define OSALMTestlo 5

#define OSTestTskID1 1
#define OSTestTskID2 2
#define OSTestTskID3 3
#define OSTestTskIDlo 4
#define OSTestTskIDhi 0

void OS_TestSuite(void)
{

	uint8_t test0pass=0;


    OS_CreateTask(TestTaskhi, OSTestTskIDhi);
    OS_CreateTask(TestTask0, OSTestTskID1);
    OS_CreateTask(TestTask1, OSTestTskID2);
    OS_CreateTask(TestTask2, OSTestTskID3);
    OS_CreateTask(TestTasklo, OSTestTskIDlo);

    OS_CreateAlarm(OSALMTest0,OSTestTskID1);
    OS_CreateAlarm(OSALMTest1,OSTestTskID2);
    OS_CreateAlarm(OSALMTest2,OSTestTskID3);
    OS_CreateAlarm(OSALMTest3,OSTestTskID1); // testcase 10
    OS_CreateAlarm(OSALMTestlo,OSTestTskIDlo);
    OS_CreateAlarm(OSALMTesthi,OSTestTskIDhi);

	OS_StartExecution() ;
	while(1)
	{
		// THIS IS the idle task which will be preempted by all other tasks.
		// NO OS-wait-API allowed here!!!

#if OS_USEMEMCHECKS ==1
		r = OS_GetUnusedStack(0);
		s = OS_GetUnusedStack(1);
		t = OS_GetUnusedStack(2);
		u = OS_GetUnusedStack(3);
#endif

		switch(testcase)
			{
				case 0:
					// Task Priority
					if(!test0pass)
					{
						assert(testvar ==3);
						test0pass = 1;
					}
					break;
				default:
					break;
			}
	}

}

volatile uint32_t testtime=0;


void TestTask1(void)
{
	uint8_t ret,i;
	uint32_t ts,te;
	uint8_t v;
	uint16_t t=25;
	uint32_t ti,tj;

	while(1)
	{
		switch(testcase)
		{
			case 0:
				// Task Priority
				assert(testvar ==1);
				testvar++;
				break;
			case 1:
				OS_WaitTicks(OSALMTest1, 1);
				// Timer / Wait
				ti = OS_GetTicks(); // get time
				OS_WaitTicks(OSALMTest1, t);
				tj = OS_GetTicks(); // get time
				assert(tj-ti == t); // waited time was correct
									
				break;
			case 2:
				// WaitEvent
				// SetEvent
				// wait for A, then A occurs
				testvar=1;
				OS_WaitEvent(1<<0);
				testvar=2;
				// A occurs, then wait for A
				WasteOfTime(50);
				assert(testvar==3);
				OS_WaitEvent(1<<0);
				testvar=4;

				// wait for more events, one occurs, then the other
				ret = OS_WaitEvent(1<<1 | 1<<2);
				testvar=5;
				assert(ret == (1<<2));
				testvar=6;
				ret = OS_WaitEvent(1<<1 | 1<<2);
				testvar=7;
				assert(ret == (1<<1));
				testvar=8;

				// wait for less events

				break;
			case 3:
				// MutexGet
				// MutexRelease			
				// two cases: 
				// 1:mutex is free
				OS_MutexGet(0);
				WasteOfTime(50);
				testvar = 1;
				OS_WaitTicks(OSALMTest1, 50);
				assert(testvar == 1);
				OS_MutexRelease(0);

				OS_WaitEvent(1<<1);
				// 2:mutex is occupied by lower prio
				OS_WaitTicks(OSALMTest1, 50);
				OS_MutexGet(0);
				testvar = 3;
				WasteOfTime(50);
				assert(testvar == 3);
				OS_MutexRelease(0);

				break;
			case 4:
				// QueueIn
				// QueueOut

				TestQ.read = 13; // tweak the queue to a "used" state
				TestQ.write =13;
				ret = OS_QueueOut(&TestQ,&v); // q empty at start
				assert(ret == 1);
				OS_WaitEvent(1<<6);
				for (i=0;i<20;i++) // forward
				{
					ret = OS_QueueOut(&TestQ,&v);
					assert(v==i);
					assert(ret ==0);
				}
				// Q empty
				ret = OS_QueueOut(&TestQ,&v);
				//assert(v==i);
				assert(ret ==1);
				OS_WaitEvent(1<<6);
				for (i=20;i>0;i--) // backward
				{
					ret = OS_QueueOut(&TestQ,&v);
					assert(v==i);
					assert(ret ==0);
				}
				// Q empty
				ret = OS_QueueOut(&TestQ,&v);
				
				assert(ret ==1);
				//OS_WaitEvent(1<<6);

				break;
			case 5:
				// SetAlarm
				// WaitAlarm
				ts = OS_GetTicks();
				OS_WaitAlarm(OSALMTest1);
				te = OS_GetTicks();
				assert(te-ts == 53);
				break;
			case 6:
				// Event with timeout
				ret = OS_WaitEventTimeout(1<<3,OSALMTest1, 30);
				assert(ret == 1<<3); // no timeout, it was the event.


				ti = OS_GetTicks(); // get time
				ret = OS_WaitEventTimeout(1<<3,OSALMTest1, t); // wait for the event, which never comes...
				assert(ret == 0); // timeout recoginzed
				tj = OS_GetTicks(); // get time
				assert(tj-ti == t); // waited time was correct


				break;
			case 7:
				// long queue


				break;
			default:
				break;
		}
	OS_WaitEvent(1<<7);
	}// while(1)

}

void TestTask2(void)
{
	uint32_t ts,te;
	uint32_t i,j;
	uint16_t t;
	while(1)
	{
		switch(testcase)
		{
			case 0:
				// Task Priority
				assert(testvar ==2);
				testvar++;
				break;
			case 1:
				// Timer / Wait
				OS_WaitTicks(OSALMTest2, 2);
				t=50;
				i = OS_GetTicks(); // get time
				OS_WaitTicks(OSALMTest2, t);
				j = OS_GetTicks(); // get time
				assert(j-i == t); // waited time was correct
				break;
			case 2:
				// WaitEvent
				// SetEvent

				break;
			case 3:
				// MutexGet
				// MutexRelease			
				// two cases: 
				// 1:mutex is occupied by higher prio
				WasteOfTime(10);
				OS_MutexGet(0);
				testvar = 7;
				WasteOfTime(50);
				assert(testvar == 7);
				OS_MutexRelease(0);
				OS_SetEvent(OSTestTskID2,1<<1);
				// 2:mutex is occupied
				OS_MutexGet(0);
				testvar = 4;
				WasteOfTime(50);
				assert(testvar == 4);
				OS_MutexRelease(0);

				break;
			case 4:
				// QueueIn
				// QueueOut

				break;
			case 5:
				// SetAlarm
				// WaitAlarm
				ts = OS_GetTicks();
				OS_WaitAlarm(OSALMTest2);
				te = OS_GetTicks();
				assert(te-ts == 44);

				break;
			case 6:
				// Event with timeout


				break;
			case 7:
				// long queue


				break;
			default:
				break;
		}
	OS_WaitEvent(1<<7);
	}// while(1)

}


// This is the coordinator task, the main beat for the tests comes from here.
void TestTask0(void)
{
	uint8_t ret,i;
	uint32_t ti,tj;
	uint16_t t;

	while(1)
	{
		switch(testcase)
		{
			case 0:
				// Task Priority
				assert(testvar ==0);
				testvar++;
				break;
			case 1:
				// Timer / Wait
				OS_WaitTicks(OSALMTest0, 3);
				t=10;
				ti = OS_GetTicks(); // get time
				OS_WaitTicks(OSALMTest0, t);
				tj = OS_GetTicks(); // get time
				assert(tj-ti == t); // waited time was correct
				break;
			case 2:
				// WaitEvent
				// SetEvent
				// wait for A, then A occurs
				testvar =0;
				OS_WaitTicks(OSALMTest0, 10);
				assert(testvar==1);
				OS_SetEvent(OSTestTskID2,1<<0);
				OS_WaitTicks(OSALMTest0, 2);
				assert(testvar==2);
// A occurs, then wait for A
				testvar =3;
				OS_SetEvent(OSTestTskID2,1<<0);
				assert(testvar==3);
				OS_WaitTicks(OSALMTest0, 100); // other task wastes time...
				assert(testvar==4);
// wait for more events, one occurs, then the other
				OS_SetEvent(OSTestTskID2,1<<2);
				assert(testvar==4);
				OS_WaitTicks(OSALMTest0, 10);
				assert(testvar==6);
				OS_SetEvent(OSTestTskID2,1<<1);
				assert(testvar==6);
				OS_WaitTicks(OSALMTest0, 10);
				assert(testvar==8);

				break;
			case 3:
				// MutexGet
				// MutexRelease			

				break;
			case 4:
				// QueueIn
				// QueueOut
				OS_WaitTicks(OSALMTest0, 50);
				for (i=0;i<20;i++) // forward
				{
					ret = OS_QueueIn(&TestQ,&i);
					assert(ret==0);
				}
				OS_SetEvent(OSTestTskID2,1<<6);
				OS_WaitTicks(OSALMTest0, 50);
				for (i=20;i>0;i--) // backward
				{
					ret = OS_QueueIn(&TestQ,&i);
					assert(ret==0);
				}
				OS_SetEvent(OSTestTskID2,1<<6);
				OS_WaitTicks(OSALMTest0, 50);
				
				// check, if q is empty
				t = OS_GetQueueSpace(&TestQ);
				assert(t==64);
					
								
				for (i=0;i<64;i++) // overload the Q
				{
					ret = OS_QueueIn(&TestQ,&i);
					assert(ret==0);
				}
				
				// check, if none left
				t = OS_GetQueueSpace(&TestQ);
				assert(t==0);				
				
				// now one too much:
				ret = OS_QueueIn(&TestQ,&i);
				assert(ret==1);

				break;
			case 5:
				// SetAlarm
				// WaitAlarm
				OS_SetAlarm(OSALMTest1,53);
				OS_SetAlarm(OSALMTest2,44);
				break;
			case 6:
				// Event with timeout
				OS_SetEvent(OSTestTskID2,1<<3);
				OS_WaitTicks(OSALMTest0, 50);
				// do not set the second one...
				
				break;
			case 7:
				//queue long test
				ret = OS_QueueOut(&TestQLong,(uint8_t*)&ti); // Q empty
				assert(ret == 1);
				for (i=0;i<10;i++)
				{
					ret = OS_QueueIn(&TestQLong,(uint8_t*)&longQtest[i]);
					assert (ret==0);
				}
				ret = OS_QueueIn(&TestQLong,(uint8_t*)&longQtest[9]);
				assert (ret==1); // Q full
				for (i=0;i<10;i++)
				{
					ret = OS_QueueOut(&TestQLong,(uint8_t*)&ti);
					assert(ret==0);
					assert(ti == longQtest[i]);
				}
				ret = OS_QueueOut(&TestQLong,(uint8_t*)&ti);
				assert(ret==1); // q empty

				break;
			case 8:
				// Nested Mutexes
				OS_MutexGet(0);
				WasteOfTime(1);
				OS_MutexGet(1);
				WasteOfTime(1);
				OS_MutexRelease(0);
				WasteOfTime(1);
				OS_MutexRelease(1);
				assert(1==1); // if it doesnt work, we do not come to here,
				
				break;
			case 9:
				// test of OS_GetUnusedStack (uint8_t TaskID)

				t=OS_GetUnusedStack(OSTestTskID1); // task0
				assert(t>80 && t<180);

				t=OS_GetUnusedStack(OSTestTskID2); // task1
				assert(t>80 && t<180);

				t=OS_GetUnusedStack(OSTestTskID3); // task2
				assert(t>80 && t<180);

	/* useless big			t=OS_GetUnusedStack(OS_NUMTASKS); // idle
				assert(t>2000 && t<2600); */


				break;
			case 10:
				ti = OS_GetTicks();
				OS_SetAlarm(OSALMTest0,39);
				OS_SetAlarm(OSALMTest3,64);
				OS_WaitAlarm(OSALMTest0);
				tj = OS_GetTicks();
				assert(tj-ti == 39);
				OS_WaitAlarm(OSALMTest3);
				tj = OS_GetTicks();
				assert(tj-ti == 64);

				break;
			case 11:
				assert(0); // just for proofing that it works!

				break;
			default:
				// The END
				while(1)
				{
					// sit here and wait for someone picking up the results out of "TestResults". 0 = OK. 
					// "TestProcessed" shows the number of processed assertions.
					asm("breakpoint");
				}
				break;
		}

		OS_WaitTicks(OSALMTest0, 500); // wait for tests to be processed...

		testcase++;
		OS_SetEvent(OSTestTskID2,1<<7); // tick other tasks in sync.
		OS_SetEvent(OSTestTskID3,1<<7);
	}//while(1)


}

void TestTaskhi(void)
{
	while(1)
	{
		OS_SetAlarm(OSALMTesthi,3);
		OS_WaitAlarm(OSALMTesthi);
	}
}

void TestTasklo(void)
{
	while(1)
	{	

		OS_SetAlarm(OSALMTestlo,3);
		OS_WaitAlarm(OSALMTestlo);
		//clutterRegs(); // call asm function, which "paints" all registers // todo rein oder raus?
	}
}


static WasteOfTime(uint32_t waittime)
{
	volatile uint32_t oldTime,newTime;
	oldTime = OS_GetTicks(); // get time
	do
	{
		newTime = OS_GetTicks(); // get time
	}
	while (newTime - oldTime < waittime);
}


#endif // OS_DO_TESTSUITE == 1

