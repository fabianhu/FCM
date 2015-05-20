/*
 * testsuite.c
 * FCM test coordination, will not be run in "fly"
 *
 * Created: 25.04.2014 16:55:40
 *
 * (c) 2014-2015 by Fabian Huslik
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
#include <fastmath.h>
#include "../FabOS_config.h"
#include "../FabOS/FabOS.h"
#include "vector.h"
#include "GPS.h"
#include "governing.h"
#include "NAV.h"
#include "../menu/menu.h"
#include "../menu/menu_variant.h"
#include "../config.h"
#include "TaskControl.h"
#include "TaskLED.h"
#include "testsuite.h"
#include "quaternions/quaternions.h"

#if TEST_RUN == 1

uint8_t testcase = 0; // global test case number
uint8_t TestResults[MAXTESTCASES]; // test result array (0= OK)
uint8_t TestProcessed[MAXTESTCASES]; // test passed array (number of processed assertions

void test_run(void)
{
	LED_BLUE_ON;
	
	// set parameters:
	//myPar.nav_max_accel.sValue = 2;	// The max acceleration out of nav in ms^2  (2)
	//myPar.nav_acc_flt_glob.sValue = ;	// global acceleration filter value (1000) 1000 i
	//myPar.nav_alpha_H.sValue = ;	// speed filter value (9990) 10000 is fast prio
	//myPar.nav_alpha_speed.sValue = ;	// speed filter value (9990) 10000 is fast prio
	//myPar.nav_alpha_Pos.sValue = ;	// speed filter value (9990) 10000 is fast prio
	
	//////////////////////////////////////////////////////////////////////////
	testcase = 0;
	//TheBigTest();
	
	testcase = 1;
	quaternion_test();
	
	testcase = 2;
	test_GPS_calcHeading();

	//////////////////////////////////////////////////////////////////////////
	testcase = 3;
	test_GetSetSpeed();

	//////////////////////////////////////////////////////////////////////////
	testcase = 4;
	test_GetBankAndThrustFromAccel();


	//////////////////////////////////////////////////////////////////////////
	testcase = 5;	
	test_PIDf();


	//////////////////////////////////////////////////////////////////////////
	testcase = 6;
	// The GPS stuff
					  	// 	  483827900 , 108526700 // home !!
	gps_coordinates_t gpssrc ={483829100 , 108527100}; // lat , lon in 10 000 000 th degrees = FCM "Home"
	gps_coordinates_t gpsdst ={484101200 , 108322700}; // 333° , 3381 m = Täfertingen
	gps_coordinates_t gpsdst2={483902900 , 108661700}; // ca. 55° , 1289 m = Funkturm
	
	volatile float distx_m,disty_m;
	static float dist_m;
	volatile float heading_deg;
	
	GPS_distance_xy_m(gpssrc,gpsdst,&distx_m,&disty_m);
	
	dist_m = vector2len(distx_m,disty_m);
	heading_deg = GPS_calcHeading(distx_m, disty_m)*radgra;
		
	assert(fabs(heading_deg - 333)<3.0); // measured 333°
	assert(fabs(dist_m-3381)< 10); // measured 
	
	GPS_distance_xy_m(gpssrc,gpsdst2,&distx_m,&disty_m);
	
	dist_m = vector2len(distx_m,disty_m);
	heading_deg = GPS_calcHeading(distx_m, disty_m)*radgra;
	
	assert(fabs(heading_deg - 55)<10.0); // estimated ca. 55°
	assert(fabs(dist_m-1289)< 10); // measured
	
	
	//////////////////////////////////////////////////////////////////////////
	testcase = 7;
	// The random stuff	
	
	vector3_t vsubtest1 = {1.2,3.4,5.6};
	vector3_t vsubtest2 = {1.2,3.4,5.6};
	vector3_t vsubtest3 = {0,0,0};
	
	vsubtest3 = vector_subtract(&vsubtest1,&vsubtest2); // get the 3D-vector to the target
	
	assertfequal(vsubtest3.x,0);
	assertfequal(vsubtest3.y,0);
	assertfequal(vsubtest3.z,0);
	
	//////////////////////////////////////////////////////////////////////////
	testcase = 8;
	// GPS and distance stuff
	
	NAV_SetOrigin_xy_cm(gpssrc);
	NAV_SetOrigin_z_m(450.0);
	static vector2_t resvec;
	resvec = NAV_ConvertGPS_to_m(gpssrc);
	assertfequal(resvec.x,0);
	assertfequal(resvec.y,0);
	
	
	resvec = NAV_ConvertGPS_to_m(gpsdst); // 333° , 3381 m
	assert(fabs(resvec.x + 1511)<3);
	assert(fabs(resvec.y - 3029)<3);
	
	//////////////////////////////////////////////////////////////////////////
	testcase = 9;
	// GPS position update
	myPar.cal_gps_filter.sValue = 990; // almost no filter
	
	uint32_t n;
	
	n = 1000;
	while (n--)
	{
		NAV_UpdatePosition_xy(gpsdst);
	}

	vector3_t sim_acc = {0,0,0};
	vector3_t pos_res = {0,0,0};
	vector3_t vel_res = {0,0,0};
	
	n = 1000;	
	while (n--)
	{	
		//Superfilter(sim_acc,&pos_res,&vel_res);
	}

	assert(fabs(pos_res.x + 1511)<3);
	assert(fabs(pos_res.y - 3029)<3);
	
	//////////////////////////////////////////////////////////////////////////
	testcase = 10;
	// Nav Governor
	//test_NAV_Governor();
	
	
	//////////////////////////////////////////////////////////////////////////
	testcase = 11;
	
	volatile quaternion_t testquat;
	
	testquat = quaternion_from_euler(33.0/radgra,44.0/radgra,55.0/radgra);
	
	if(testquat.w <0)
		quaternion_flip(&testquat);
	
	assertfequal(testquat.w,0.73942);
	assertfequal(testquat.x,0.44019);
	assertfequal(testquat.y,-0.06772);
	assertfequal(testquat.z,0.50487);
	
	volatile float tx,ty,tz;
	quaternion_to_euler(testquat,&tx,&ty,&tz);
	assertfequal(tx*radgra,33.0);
	assertfequal(ty*radgra,44.0);
	assertfequal(tz*radgra,55.0);
	
	testquat = quaternion_from_euler(-33/radgra,-44/radgra,-55/radgra);
	quaternion_to_euler(testquat,&tx,&ty,&tz);
	assertfequal(tx,-33/radgra);
	assertfequal(ty,-44/radgra);
	assertfequal(tz,-55/radgra);
	
	
	//////////////////////////////////////////////////////////////////////////
	testcase = 11;
	// sensor fusion
	
	
	//////////////////////////////////////////////////////////////////////////
	// test evaluation
	LED_BLUE_OFF;
	if(testcase >= MAXTESTCASES)
	{		
		asm("breakpoint");
		LED_RED_ON;
		while(1);
	}
	
	
	
	for (int i = 0; i<testcase;i++)
	{
		if(TestResults[i] !=0)
		{
			asm("breakpoint");
			LED_RED_ON;
			while(1);
		}
	}

	LED_GREEN_ON;
	
	asm("breakpoint"); // all tests passed.
	
}



#endif //TEST_RUN == 1


