/*
 * gyro.c
 *
 * Created: 26.02.2012 11:58:48
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
#include "modules/types.h"
#include "modules/vector.h"
#include "gyro.h"
#include "modules/spi.h"
#include "l3gd20.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/emergency.h"
#include "modules/governing.h"
#include "config.h"
#include "menu/menu.h"
#include "menu/menu_variant.h"


void gyro_init(void)
{
	OS_WaitTicks(OSALM_CTRLWAIT,10);// calm down everything ;-)

	l3gd20_CheckWhoAmI();	

	// initialize the queue for configuration of the sensors
	l3gd20_ConfigInitQueue();
		
	SPI_startqueue(); 
	OS_WaitTicks(OSALM_CTRLWAIT,200);// wait for success.
	
	l3gd20_ConfigReadQueue();

	SPI_startqueue(); 
	OS_WaitTicks(OSALM_CTRLWAIT,10);// wait for success.

	l3gd20_Enable_ISR_transfer(); // after this, no more SPI_startqueue!!!
	
	SPI_startqueue(); // to wake up the Gyro processing
	
}


int32_t gyro_getValues_flt(vector3_t* val)
{
	int32_t ret;
	int32_t xr,yr,zr;
	static int32_t s_flt_x_mem, s_flt_y_mem, s_flt_z_mem; // static to store filtered values
	
	ret = l3gd20_getValues_raw(&xr,&yr,&zr); // exchange here to the matching component.

	val->x = Filter_mem(&s_flt_x_mem,xr,myPar.cal_gyro_filter.sValue);
	val->y = Filter_mem(&s_flt_y_mem,yr,myPar.cal_gyro_filter.sValue);
	val->z = Filter_mem(&s_flt_z_mem,zr,myPar.cal_gyro_filter.sValue);
	
	return ret;
}


#define GYROCALFILTER 20

void gyro_calibrate(vector3_t* cal)
{
	volatile int32_t rxc=0,ryc=0,rzc=0;
	volatile int32_t cxc=0,cyc=0,czc=0;
	volatile int32_t Rfx, Rfy, Rfz; // volatile for this function!!!!! (wrong compiler behaviour.)
	
	OS_WaitTicks(OSALM_CTRLWAIT,10);
		
	l3gd20_getValues_raw(&rxc,&ryc,&rzc);

	Rfx=rxc*GYROCALFILTER; //init filter stuff 
	Rfy=ryc*GYROCALFILTER;
	Rfz=rzc*GYROCALFILTER;
		
	for (uint32_t i = 0; i< 200;i++) // loop for 2s
	{
	
		OS_WaitTicks(OSALM_CTRLWAIT,10);
		
		if(l3gd20_getValues_raw(&rxc,&ryc,&rzc) != 0)
		{
			emstop(2);
		}
		
		cxc = Filter_mem(&Rfx,rxc,GYROCALFILTER);
		cyc = Filter_mem(&Rfy,ryc,GYROCALFILTER);
		czc = Filter_mem(&Rfz,rzc,GYROCALFILTER);
	}	
	
	cal->x = cxc;
	cal->y = cyc;
	cal->z = czc;
}

