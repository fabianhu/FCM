/*
 * l3gd20.c
 *
 * Created: 26.02.2012 09:50:05
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
#include "modules/eic.h"
#include "l3gd20.h"
#include "modules/spi.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include <fastmath.h>

typedef enum
{
	eReinit_NormalOp,
	eReinit_Requested,
	eReinit_Processed,
	eReinit_Fail
}eReInitState_t;

void l3gd20_Adress(uint8_t addr, uint8_t mult, uint8_t read);
void l3gd20_Data(uint8_t data);
int16_t l3gd20_ConvertResult(uint8_t msb, uint8_t lsb);
void l3gd20_getValues_inDMA_ISR(void);
int l3gd20_getValues_fromSPIQ(volatile int32_t* x, volatile int32_t* y, volatile int32_t* z, volatile int32_t* state);

static uint32_t ResultQueueStartIdx;
eReInitState_t ReInitState;


void l3gd20_ConfigInitQueue(void)
{
	SPI_ResetQ();
	l3gd20_Adress(L3GD20_CTRL_REG1,1,0);
	l3gd20_Data(L3GD20_CTRL_REG1_SET); // see l3gd20.h
	l3gd20_Data(L3GD20_CTRL_REG2_SET);
	l3gd20_Data(L3GD20_CTRL_REG3_SET);
	l3gd20_Data(L3GD20_CTRL_REG4_SET);
	l3gd20_Data(L3GD20_CTRL_REG5_SET);
	SPI_EndQ();
	
}

void l3gd20_ConfigReadQueue(void)
{
	SPI_ResetQ(); 
	l3gd20_Adress(L3GD20_OUT_TEMP,1,1); // mult / read
	l3gd20_Data(0); // dummy output temp
	l3gd20_Data(0); // dummy output	status
	l3gd20_Data(0); // dummy output	xL
	l3gd20_Data(0); // dummy output	xH
	l3gd20_Data(0); // dummy output	yL
	l3gd20_Data(0); // dummy output	yH
	l3gd20_Data(0); // dummy output	zL
	l3gd20_Data(0); // dummy output zH
	SPI_EndQ();
}

void l3gd20_CheckWhoAmI(void) // before activation of ISR based process!!!
{
	SPI_ResetQ(); // todo do outside!!
	l3gd20_Adress(L3GD20_WHO_AM_I,0,1); // mult / read
	l3gd20_Data(0); // dummy output
	SPI_EndQ(); // todo do outside!!
	
	SPI_startqueue();
	OS_WaitTicks(OSALM_CTRLWAIT,5);
	if (SPI_resQ[1] != 0b11010100) 
	{
		// we will not fly today.
		asm("breakpoint");
		OS_ShutdownHook(3); // no better idea
	}
	
	// self test todo
	// switch on ST 0 ( Ctrl Reg4 = 0bxxxxx01x
	// check + values
	
	// self test
	// switch on ST 1 ( Ctrl Reg4 = 0bxxxxx11x
	// check - values
	
	// switch off ST
	
	// ready to go, sensor is good.		
}

uint32_t debug_Gy_INcomplete, debug_Gy_complete;

int l3gd20_getValues_fromSPIQ(volatile int32_t* x, volatile int32_t* y, volatile int32_t* z, volatile int32_t* state)
{
	uint32_t temperature;
	temperature = SPI_resQ[ResultQueueStartIdx]; 
	*state = SPI_resQ[ResultQueueStartIdx+1]; 
	
	if((*state & 0b00000111) == 0b00000111) // all axes received.
	{
		// obfuscation of signals due to reverse mounting. correct for axes in FCM manual.
		*x = l3gd20_ConvertResult(SPI_resQ[ResultQueueStartIdx+3],SPI_resQ[ResultQueueStartIdx+2]); // sensors x
		*y = -l3gd20_ConvertResult(SPI_resQ[ResultQueueStartIdx+5],SPI_resQ[ResultQueueStartIdx+4]); // sensors y
		*z = -l3gd20_ConvertResult(SPI_resQ[ResultQueueStartIdx+7],SPI_resQ[ResultQueueStartIdx+6]); // sensors z
		debug_Gy_complete++;
		return 0;
	}
	else
	{
		debug_Gy_INcomplete++;
		//SPI_startqueue(); // try again... geht schon irgendwie, braucht aber min 2 Versuche jedes Mal!
		//SPI_ReInit(); // die Lösung ist, der SPI wird vom Flashen abgewürgt...
		return 1;
	}	
		
}



int16_t l3gd20_ConvertResult(uint8_t msb, uint8_t lsb)
{
	uint16_t ret;
	
	ret = (msb<<8) | lsb;
	
	return *((uint16_t*)&ret)	; // do NOT cast!
}



void l3gd20_Adress(uint8_t addr, uint8_t mult, uint8_t read)
{
	uint8_t d;
	d= (read<<7)| // !write
		(mult<<6)|
		(addr);
	ResultQueueStartIdx = SPI_AddQ8(d,CHIPSEL_L3GD20,0); // address 
}

void l3gd20_Data(uint8_t data)
{
	SPI_AddQ8(data,CHIPSEL_L3GD20,0);
}



__attribute__((__interrupt__))
static void eic_int_handler_GYRO(void) // the interrupt signal from l3gd20 is high as long the data z register is not read. (85µs)
{
	eic_clear_int(6);
	
	
	// trigger next pull.
	SPI_startqueue();
		
}



void l3gd20_Enable_ISR_transfer(void)
{
	// init ISR
	INTC_register_interrupt(&eic_int_handler_GYRO, AVR32_EIC_IRQ_6, AVR32_INTC_INTLEVEL_INT2);
	
	SPI_RegisterReadyHandler(l3gd20_getValues_inDMA_ISR);
	
	// activate and enable EIC pins:
	eic_enable_channel(6); // enable rising edge isr.
	
	
}





#define GYROARRSIZE 25
volatile gyro_t gyro_array[GYROARRSIZE];
volatile uint8_t gyro_array_read=0; // next pos to be read
volatile uint8_t gyro_array_write=0; // next pos to be written  // Q is empty, if both are equal.

uint32_t debug_Gy_RECONFIG=0;

void l3gd20_getValues_inDMA_ISR(void)
{
	int ret;

	switch (ReInitState)
	{
		case eReinit_NormalOp:
			// thread safe, can not be interrupted, since in ISR.
			if(gyro_array_write+1 == gyro_array_read || (gyro_array_read==0 && gyro_array_write+1 == GYROARRSIZE))
			{
				// q full
				return; // just leave it, the consumer has to consume first, I can only append here, otherwise data, which is worked, could be overwritten.
			}
	
			ret = l3gd20_getValues_fromSPIQ(&gyro_array[gyro_array_write].x,&gyro_array[gyro_array_write].y,&gyro_array[gyro_array_write].z, &gyro_array[gyro_array_write].state);
			static uint8_t test =0;
			if (ret == 0)
			{
				if(gyro_array_write == GYROARRSIZE-1)
					gyro_array_write = 0;
				else
					gyro_array_write++;
				test=0;
				break; // todo enable re-init from external source...
			}
			else
			{
				// do not increment on invalid measurement.
				test++;
				if(test > 10)
				{
					ReInitState = eReinit_Requested;
					// no break, rush through.
				}
				else if(test > 5)
				{
					// SPI needs to be reset after a flash action, which causes the SPI to be unstable.
					SPI_ReInit(); // the SPI is somewhat disturbed by the flash routine or sth... ??? 
					test=0;
					break;
				}
				else
				{		
					break;
				}

			}		
			 // break conditional!!!!
		case eReinit_Requested:
			// setup Q for init sequence
			l3gd20_ConfigInitQueue();
			// and fire it
			SPI_startqueue();
			ReInitState = eReinit_Processed; // set next state
		
			break;
		case eReinit_Processed:
			// restore q for data get sequence
			l3gd20_ConfigReadQueue();
		
			if(gpio_pin_is_high(AVR32_PIN_PB02)) // check, if ISR line is high, then fire
			{
				SPI_startqueue();
			}
			
			debug_Gy_RECONFIG++;
			
			// debug cpu_delay_ms(100,BOARD_SYS_HZ);;
			
			ReInitState = eReinit_NormalOp;
		
			break;
		case eReinit_Fail:

			break;
		default:
			break;
	}


		
}

uint32_t debug_GY_NoMeasure=0;

int l3gd20_getValues_raw(int32_t* x, int32_t* y, int32_t* z)
{
	volatile int32_t i=0;
	volatile int32_t xflt=0;
	volatile int32_t yflt=0;
	volatile int32_t zflt=0;
	volatile uint32_t rd,wr; // init ALL variable at first, sum up did not work, if declared later.

	// filter the values out of the ring buffer
	//OS_DISABLEALLINTERRUPTS;// should be no problem: 
	wr = gyro_array_write; // atomic copy for thread safety; Access to all other variables in array is safe.
	rd = gyro_array_read; // atomic copy for thread safety; Access to all other variables in array is safe.

	while(rd!=wr) // read until empty
	{
		if((gyro_array[rd].state & 0b00000111) == 0b00000111)
		{
			i++;

			xflt += gyro_array[rd].x;
			yflt += gyro_array[rd].y;
			zflt += gyro_array[rd].z;
		}


		rd++;
		if(rd==GYROARRSIZE)
			rd=0;
	}
	gyro_array_read = rd; // atomic copy for thread safety; Access to all other variables in array is safe.*/
	
	static uint32_t cnt_empty_workaround=0;
	
	if (i==0)// div by 0  / Q was empty.
	{
		debug_GY_NoMeasure++;
		cnt_empty_workaround++;
		if(cnt_empty_workaround > 5) // dirty workaround, as flashing disturbs the SPI...
		{
			SPI_startqueue();
			cnt_empty_workaround = 0;

		}
		*x = 0;
		*y = 0;
		*z = 0;		
		//OS_ENABLEALLINTERRUPTS;// should be no problem:
		return 1; 
	}		

	*x = xflt / i;
	*y = yflt / i;
	*z = zflt / i;

	 //OS_ENABLEALLINTERRUPTS;// should be no problem:
	 cnt_empty_workaround = 0;

	 return 0;
}

// return rad/s
float l3gd20_raw_to_rad(int32_t raw)
{
	float fraw = raw;
	return fraw*0.07*2*M_PI/360; // 0.07 = 70 mdps/digit from datasheet
}