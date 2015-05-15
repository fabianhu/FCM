/*
 * TaskSens.c
 *
 * Using the i2c interface
 *
 * Created: 10.10.2012 23:15:09
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


#include "asf.h"
#include "modules/types.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/twi.h"
#include "modules/bmp085.h"
#include "modules/vector.h"
#include "TaskTWI.h"
#include "lsm303dlhc.h"
#include "modules/eic.h"
#include "TaskControl.h"
#include "TaskLED.h"
#include "modules/vector.h"
#include "modules/governing.h"
#include "config.h"
#include "menu/menu.h"
#include "menu/menu_variant.h"
#include "modules/GPS.h"
#include "modules/NAV.h"
#include "modules/emergency.h"

void TaskTWI(void);
void TaskTWIWait1ms(void);
void TWIreadyhandler(void);
__attribute__((__interrupt__)) static void int_handler_ACC(void);
uint32_t BaroMachine(void);
int32_t twi_getAccel_raw(vector3_t* res); // todo wo kommen die denn her?
int32_t twi_getMagnet_raw(vector3_t* res);

typedef enum
{
	etwi_Idle,
	etwi_readACC,
	etwi_readMAG,
	etwi_BAR
}eTWIState_t;

typedef enum
{
	eBMP_Idle,
	eBMP_waitTemp,
	eBMP_waitBaro,
	eBMP_readTemp,
	eBMP_readBaro
}eBMP085State_t;

//uint16_t ut;
//uint32_t up;
int32_t bmp085_temp;
unsigned long bmp085raw; // the raw value of baro out of ISR
int32_t s_nHeight_mm;
//int16_t temperature;
//int32_t Pressure_Pa;
//uint32_t waittime_ms;

uint32_t lastACCSample;
eTWIState_t TWIstate;

void TaskTWIWait1ms(void)
{
	OS_WaitTicks(OSALM_TWIWAIT,2); // for test reasons we wait 2 ms
}

#define TWINOISRMODE 0 // was used for development; no more used.

void TaskTWI(void)
{
	
	OS_WaitTicks(OSALM_TWIWAIT,10); // BMP085 needs 10ms in advance.
	TWI_init();
	bmp085_init(TaskTWIWait1ms);
	
	lsm303_config_accel(TaskTWIWait1ms);
	lsm303_config_magnet(TaskTWIWait1ms);	
	
	// init ISR
	INTC_register_interrupt(&int_handler_ACC, AVR32_EIC_IRQ_0, AVR32_INTC_INTLEVEL_INT2);
	// activate and enable EIC pins:
	eic_enable_channel(0); // enable rising edge isr.
	
	TWI_RegisterReadyHandler(TWIreadyhandler);
	
	lsm303_TWI_trig_read_accel(); // restart interrupt
	TWIstate = etwi_readACC;
	lastACCSample = OS_GetTicks();
		
	while(1)
	{
		if (OS_GetTicks()-lastACCSample > 400)
		{
			lsm303_TWI_trig_read_accel(); // restart interrupt
			TWIstate = etwi_readACC;
		}
		
		uint8_t ret = OS_WaitEventTimeout(OSEVT_TWIRDY,OSALM_TWITIMEOUT,200);
		if((ret & OSEVT_TWIRDY) == 0)
		{
			// timeout
			//asm("breakpoint"); fixme wtf why and why
			//emstop(5); // will hang up while flashing, if we stop here!
		}
		else
		{
			long bar = bmp085_calc_pressure(bmp085raw); // just read stuff from rx buffers out of ISR!!! (large calculation!)
			int32_t h_mm = bmp085_calcHeight_mm(bar);
			s_nHeight_mm = h_mm; // update global
			#if  SIMULATION == 1
				// do not update z position, this is done in TaskNavi to make it even more complicated. ;)
			#else
				NAV_UpdatePosition_z_m((float)h_mm*0.001);
			#endif
		}
		
	}
}


typedef struct results_tag // only used within this file!!
{
	vector3_t v;
	int32_t state;
}results_t;


#define ACC_QSIZE 20
#define MAG_QSIZE 10

results_t mag_res[MAG_QSIZE];
results_t acc_res[ACC_QSIZE];
volatile uint8_t acc_rd = 0;
volatile uint8_t acc_wr = 0;
volatile uint8_t mag_rd = 0;
volatile uint8_t mag_wr = 0;


static eBMP085State_t BMPState = eBMP_Idle;
static uint32_t StartTime;
static uint32_t WaitTime;
static uint8_t TempDecrementer=0;

#define GETTEMPEVERY 10
	
uint32_t BaroMachine(void) // return 1 if it has triggered TWI
{
	

	// if nothing is running, start a conversion
	// if conversion was started and conversion time is elapsed, start getting results
	uint32_t now = OS_GetTicks();
	uint32_t elapsed = now - StartTime;
	switch(BMPState)
	{
		case eBMP_Idle:
			if(TempDecrementer-- == 0)
			{
				TempDecrementer = GETTEMPEVERY;
				BMPState = eBMP_waitTemp;
				WaitTime = bmp085_TWI_trig_temp_meas(); // takes up to 5 ms
				StartTime = now;
				return 1; // TWI write triggered
			}
			else
			{
				BMPState = eBMP_waitBaro;
				WaitTime = bmp085_TWI_trig_press_meas(); // takes up to 25.5 ms
				StartTime = now;
				return 1; // TWI write triggered
			}
			break;
		case eBMP_waitTemp:
			if (elapsed > WaitTime)
			{
				BMPState = eBMP_readTemp;
				bmp085_TWI_trig_temp_read();
				return 1; // TWI read triggered
			}
			break;
		case eBMP_waitBaro:
			if (elapsed > WaitTime+10)
			{
				BMPState = eBMP_readBaro;
				bmp085_TWI_trig_press_read();
				return 1; // TWI read triggered
			}
			break;
		case eBMP_readTemp:
			BMPState = eBMP_Idle;
			bmp085_temp = bmp085_get_temperature(); // just read stuff from rx buffers
			break;
		case eBMP_readBaro:
			BMPState = eBMP_Idle;
			bmp085raw = bmp085_read_up();
			OS_SetEventFromISR(OSTSK_TWI,OSEVT_TWIRDY);
			break;					
		default:
		
			break;
	}
	return 0; // did not trigger TWI
}


void TWIreadyhandler(void)
{
	static uint8_t tictoc=0;
	
	/*
	Procedure:
	Acc via EIC triggers next conversion (1344Hz);
	all due conversions of other sensors (mag (200Hz), press) will be chained via TWI_ready handler.
	*/
	switch(TWIstate)
	{
		case etwi_readACC:
			lastACCSample = OS_GetTicks();
			acc_res[acc_wr].state = lsm303_get_acc_results(&acc_res[acc_wr].v);
			acc_wr++;
			if(acc_wr == ACC_QSIZE) acc_wr = 0;
			
			tictoc ++;
			if(tictoc == 4)
			{
				TWIstate = etwi_readMAG;
				lsm303_TWI_trig_read_magnet();
			}			
			else if(tictoc == 5) // 1344/6 = 224Hz (Mag runs at 220)
			{
				tictoc = 0;
				if(BaroMachine()==1)
					TWIstate = etwi_BAR;
				else
					TWIstate = etwi_Idle;
			}
			else
			{	
				TWIstate = etwi_Idle;		
			}
			break;
		case etwi_readMAG:
			// do not set state!!
			mag_res[mag_wr].state = lsm303_get_mag_results(&mag_res[mag_wr].v);
			mag_wr++;
			if(mag_wr == MAG_QSIZE) 
				mag_wr = 0;
			
			TWIstate = etwi_Idle;
			break;
		case etwi_BAR:
			if(BaroMachine()==1)
				TWIstate = etwi_BAR; // should NEVER happen !
			else
				TWIstate = etwi_Idle;
			break;
		default:
			asm("breakpoint"); // wtf why do we land here
			break;
	}

	
}

__attribute__((__interrupt__))
static void int_handler_ACC(void) 
{
	eic_clear_int(0);
	
	if(TWIstate != etwi_Idle)
	{
		return;
		//asm("breakpoint");
	}		
	// 1. trigger getting of acc data
	TWIstate = etwi_readACC;	
	lsm303_TWI_trig_read_accel();

	
}

int32_t twi_getAccel_flt(vector3_t* ret)
{
// fixme convert to float also in numbers. Actually the calibration is integer, so scaling in here is limited to whole numbers!

	vector3_t raw;
	int32_t state;
	static int32_t mx=0,my=0,mz=0;
	
	state = twi_getAccel_raw(&raw);
	
	
	ret->x = Filter_mem(&mx,raw.x, myPar.cal_acc_filter.sValue);
	ret->y = Filter_mem(&my,raw.y, myPar.cal_acc_filter.sValue);
	ret->z = Filter_mem(&mz,raw.z, myPar.cal_acc_filter.sValue);
	
	return state;
}


int32_t twi_getMagnet_flt(vector3_t* ret)
{
	vector3_t raw; 
	int32_t state;
	static int32_t mx=0,my=0,mz=0;
	
	state = twi_getMagnet_raw(&raw);
		
	ret->x = Filter_mem(&mx,raw.x,myPar.cal_mag_filter.sValue);
	ret->y = Filter_mem(&my,raw.y,myPar.cal_mag_filter.sValue);
	ret->z = Filter_mem(&mz,raw.z,myPar.cal_mag_filter.sValue);
	
	return state;
}

uint32_t debug_TWI_NoRead=0;

int32_t twi_getAccel_raw(vector3_t* res) // returns m/s^2
{
	int32_t i=0;
	int32_t xflt=0;
	int32_t yflt=0;
	int32_t zflt=0;
	uint32_t rd,wr; // init ALL variable at first, sum up did not work, if declared later.
	
	wr = acc_wr; // atomic copy for thread safety; Access to all other variables in array is safe.
	rd = acc_rd; // atomic copy for thread safety; Access to all other variables in array is safe.
	
	while(rd!=wr) // read until empty
	{
		if(acc_res[rd].state == 0)
		{
			i++;
			xflt += acc_res[rd].v.x;
			yflt += acc_res[rd].v.y;
			zflt += acc_res[rd].v.z;
		}			
		rd++;
		if(rd==ACC_QSIZE)
		rd=0;
	}
	acc_rd = rd; // atomic copy for thread safety; Access to all other variables in array is safe.*/
	
	if(i==0)
	{
		debug_TWI_NoRead++;
		res->x = 0;
		res->y = 0;
		res->z = 0;
		return 1;
	}
	
	res->x = xflt / i;;
	res->y = yflt / i;;
	res->z = zflt / i;;
	return 0;
	
}

int32_t twi_getMagnet_raw(vector3_t* res) // todo scale to gauss or similar.
{
	int32_t state;
	int32_t i=0;
	int32_t xflt=0;
	int32_t yflt=0;
	int32_t zflt=0;
	uint32_t rd,wr; // init ALL variable at first, sum up did not work, if declared later.
		
	wr = mag_wr; // atomic copy for thread safety; Access to all other variables in array is safe.
	rd = mag_rd; // atomic copy for thread safety; Access to all other variables in array is safe.
		
	while(rd!=wr) // read until empty
	{
		if(mag_res[rd].state == 0)
		{
			i++;
			xflt += mag_res[rd].v.x;
			yflt += mag_res[rd].v.y;
			zflt += mag_res[rd].v.z;
		}// just ignore the misreads (there will be gaps, if measurement is at 220 Hz, but reading is at 224Hz
		rd++;
		if(rd==MAG_QSIZE)
		rd=0;
	}
	mag_rd = rd; // atomic copy for thread safety; Access to all other variables in array is safe.
		
	if(i==0)
	{
		debug_TWI_NoRead++;
		res->x = 0;
		res->y = 0;
		res->z = 0;
		state = 1;
		return state;
	}
		
	res->x = xflt / i;;
	res->y = yflt / i;;
	res->z = zflt / i;;
	state = 0;
	
	return state;
}


int32_t twi_get_h_mm(void)
{
	return s_nHeight_mm;
}

int32_t twi_get_temp(void)
{
	return bmp085_temp;
}