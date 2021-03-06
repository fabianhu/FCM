/*
 * NAV.c
 *
 * newly Created: 12.3.2014 
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
#include "NAV.h"
#include "../menu/menu.h"
#include "../menu/menu_variant.h"
#include "../config.h"
#include "testsuite.h"
#include "vector.h"
#include "governing.h"
#include "quaternions/quaternions.h"
#include "SIM.h"

// slowly updated values from GPS and baro
vector3_t NAV_slowPos_m;
gps_coordinates_t NAV_origin;
float NAV_origin_h_m;
float raw_h;

// global info tag for info purposes for communication (not to be used for governing purposes) (not atomic!)
NAVinfo_t NAV_info;

// set the origin at start, do not use as "home" or "setpoint"
void NAV_SetOrigin_xy_cm(gps_coordinates_t pos) // tested
{
	NAV_origin.lat = pos.lat;
	NAV_origin.lon = pos.lon;
}

// set the origin at start, do not use as "home" or "setpoint"
void NAV_SetOrigin_z_m() // tested
{
	NAV_origin_h_m = raw_h;
}

vector2_t NAV_ConvertGPS_to_m(gps_coordinates_t set) // tested
{
	vector2_t ret;
	volatile float x,y;

	GPS_distance_xy_m(NAV_origin ,set ,&x,&y);
	
	ret.x = x;
	ret.y = y;
	
	return ret;
}

static uint32_t lastGPSTime;

bool NAV_GPS_OK(void)
{
	#if SIMULATION ==1
	return true;
	#else
	
	uint32_t time = OS_GetTicks();
	if(time- lastGPSTime > NAVGPSTIMEOUT_ms )
	return false;
	else
	return true;
	#endif
}	
	
// update GPS measurement 
// async call after GPS message arrived.
// timeout is handled in caller (missing event)
void NAV_UpdatePosition_xy(gps_coordinates_t coords)
{
	vector2_t act_m;

	uint32_t time = OS_GetTicks();
	lastGPSTime = time; // remember the time

	
	act_m = NAV_ConvertGPS_to_m(coords); 
	
	NAV_slowPos_m.x = act_m.x;
	NAV_slowPos_m.y = act_m.y;
}

// Update barometer measurement / async call every baro measurement
void NAV_UpdatePosition_z_m(float h)
{
	raw_h = Filter_f(raw_h,h,0.1); // remember raw value for 0 calibration
	
	h = h - NAV_origin_h_m;

	NAV_slowPos_m.z = h;
}

/*
Filter (slow) position and (fast) accelometer into position.
slow values are input asynchronously via above functions.
All in global coordinates.
acc_mpss: Accerleration world based without gravity in meters / s*s
pos: gps position and barometer height
*/

vector3_t /* debug_accel,debug_gyro, debug_mag,*/debug_gov;


void Superfilter(vector3_t acc_in_mpss, vector3_t* pos_act)
{
	static vector3_t acc_fltHP_mpss={0.0,0.0,0.0};
 	static vector3_t acc_error_mpss={0.0,0.0,0.0};
	static vector3_t fltSpeed={0.0,0.0,0.0};
	static vector3_t oldPos={0.0,0.0,0.0};
	static vector3_t slowSpeed={0.0,0.0,0.0};
	static vector3_t SlowPos_m={0.0,0.0,0.0};
			
	//fixme use BrownLinearExpo() for accel filtering, maybe outside.
	
	float alfa = (float)myPar.cal_gps_filter.sValue * 0.001;
	float alfah = (float)myPar.cal_baro_filter.sValue * 0.001;
	
	SlowPos_m.x = Filter_f(SlowPos_m.x,NAV_slowPos_m.x,alfa);
	SlowPos_m.y = Filter_f(SlowPos_m.y,NAV_slowPos_m.y,alfa);
	SlowPos_m.z = Filter_f(SlowPos_m.z,NAV_slowPos_m.z,alfah);
	
	// SIMULATION is done in GPS task TaskNavi for GPS inputs; acc_mps comes in simulated as well.
	#if DISABLE_SENSOR_FUSION_GPS == 1
	// override with
	pos_act->x = SlowPos_m.x;
	pos_act->y = SlowPos_m.y;
	pos_act->z = SlowPos_m.z;
	
	#else

	static uint32_t lastFilterTime;
	uint32_t time = OS_GetTicks();
	float dt_s =  (float)(time-lastFilterTime) * 0.001;  // in seconds
	lastFilterTime = time; // remember the time

	// acceleration to speed: dSpeed = accel * dt // fast but drifty
	// speed to position: dPos = Speed * dt
	// position to speed: speed = dPos / dt   // slow but accurate

// "other" solution:
// "fly" solely after position calculated out of the acceleration based values
// correct the acceleration and position values by the GPS
// this is done by storing old estimates and comparing them to the last position, as it comes in (delayed)


// or we do the following: each one is one integration/differentiation step away:
// calculate the actual speed out of accel.
// calculate the actual speed out of filtered (with GPS) positions.
// filter these two together using complementary filter (making it responsive, even if GPS is delayed)
// calculate a position(1) with the filtered speed.
// do another complementary filter between the position(1) and the GPS position(2) (making it responsive, even it GPS is delayed)


	// speed = 0.98 * ( speed + accel * dt  ) + 0.02 * ( slowSpeed)
	float alfa_spd = (float)myPar.nav_alpha_Spd.sValue * 0.001;
	float alfa_pos = (float)myPar.nav_alpha_Pos.sValue * 0.001;
 	float alfa_accerr = (float)myPar.test_D.sValue*0.001; 
	
	vector_highpass(&acc_fltHP_mpss, &acc_error_mpss, &acc_in_mpss, alfa_accerr); // 1 = unfiltered
	
	fltSpeed.x = alfa_spd*(fltSpeed.x - acc_fltHP_mpss.x * dt_s) + (1.0-alfa_spd)*slowSpeed.x; // 1 = acc only 0 = GPS only
	fltSpeed.y = alfa_spd*(fltSpeed.y - acc_fltHP_mpss.y * dt_s) + (1.0-alfa_spd)*slowSpeed.y; 
	fltSpeed.z = alfa_spd*(fltSpeed.z - acc_fltHP_mpss.z * dt_s) + (1.0-alfa_spd)*slowSpeed.z; 
	
	// complementary filter for position
	// new pos = old pos + spd * dt
	pos_act->x = alfa_pos*(pos_act->x + fltSpeed.x * dt_s) + (1.0-alfa_pos)*(SlowPos_m.x); // 1 = acc only
	pos_act->y = alfa_pos*(pos_act->y + fltSpeed.y * dt_s) + (1.0-alfa_pos)*(SlowPos_m.y);
	pos_act->z = alfa_pos*(pos_act->z + fltSpeed.z * dt_s) + (1.0-alfa_pos)*(SlowPos_m.z);

	slowSpeed.x = (pos_act->x - oldPos.x)/dt_s; // calculate at: new pos known, and old pos not yet overwritten.
	slowSpeed.y = (pos_act->y - oldPos.y)/dt_s; 
	slowSpeed.z = (pos_act->z - oldPos.z)/dt_s; 
	
	oldPos.x = pos_act->x;
	oldPos.y = pos_act->y;
	oldPos.z = pos_act->z;
	

	
	#endif
	


}


/*
Get the desired speed setpoint out of the distance to trg.
out: speed setpoint in m/s
*/
// vector3_t GetSetSpeed(vector3_t* actPos_m, vector3_t* setPos_m) // tested
// {
// 	vector3_t ret;
// 	volatile float direction_to_dest;
// 	volatile float VMax;
// 	volatile float dist_m;
// 	volatile float speedfact;
// 	volatile float setSpeed;
// 	volatile float VMaxDist_m;	
// 	volatile float H_VMaxDist_m;
// 	volatile float H_VMax;
// 	volatile float d_height_m;
// 	volatile float h_dist;
// 	volatile float h_speedfact;
// 	volatile float h_setSpeed;
// 
// 	VMax = myPar.nav_set_speed.sValue;
// 	VMaxDist_m = myPar.nav_decel_radius.sValue;
// 	H_VMaxDist_m = myPar.nav_decel_radiush.sValue;
// 	H_VMax = myPar.nav_set_speedh.sValue;
// 	
// 	dist_m = vector2len(setPos_m->x-actPos_m->x,setPos_m->y-actPos_m->y);
// 	if(dist_m < 0.1)
// 	{
// 		// there will be no useful direction, so we just demand nothing.
// 		ret.x = 0;
// 		ret.y = 0;
// 	}
// 	else
// 	{
// 		direction_to_dest = GPS_calcHeading(setPos_m->x-actPos_m->x,setPos_m->y-actPos_m->y);
// 	
// 		dist_m = fabs(limitf(dist_m, 0, VMaxDist_m)); // distance for speed calculation
// 		speedfact = dist_m / VMaxDist_m; // speed reduction factor by distance
// 		
// 		// calculate set speed
// 		setSpeed = VMax * speedfact; // in m/s
// 
// 		// lon lat speed components:
// 		ret.x = sinf(direction_to_dest) * setSpeed;
// 		ret.y = cosf(direction_to_dest) * setSpeed;
// 	}
// 	
// 	// height has no direction, so we are fine here.
// 	H_VMaxDist_m = myPar.nav_decel_radiush.sValue;
// 	H_VMax = myPar.nav_set_speedh.sValue;
// 	
// 	// h set speed calc
// 	d_height_m = setPos_m->z - actPos_m->z;
// 	h_dist = limitf(d_height_m,-H_VMaxDist_m,H_VMaxDist_m); // distance for speed calculation
// 	h_speedfact = h_dist / H_VMaxDist_m; // speed reduction factor by distance
// 	
// 	// calculate set speed
// 	h_setSpeed = H_VMax * h_speedfact; // in m/s
// 	ret.z = h_setSpeed;
// 	
// 	return ret;
// }


/*
singleton
Navigation governor
 pos_act_m  Actual position
 target_m   target position
 *speed_act_mps actual speed

return m/s� desired acceleration
*/
static pidf_t pid_nav_x = {0,0};
static pidf_t pid_nav_y = {0,0};
static pidf_t pid_nav_z = {0,0};

// velocity related governor
// vector3_t NAV_Governor_vel( vector3_t* pos_act_m, vector3_t* target_m, vector3_t* speed_act_mps ); // unused	
// vector3_t NAV_Governor_vel( vector3_t* pos_act_m, vector3_t* target_m, vector3_t* speed_act_mps )
// {
// 	volatile vector3_t setSpeed = GetSetSpeed(pos_act_m,target_m);
// 	
// 	// PID setSpeed -> accel_command
// 	volatile vector3_t accel_command;
// 	volatile vector3_t accel_command_lim;
// 	
// 	float nav_kp = myPar.pid_nav_p.sValue*0.1;
// 	float nav_ki = myPar.pid_nav_i.sValue*0.0001;
// 	float nav_kd = myPar.pid_nav_d.sValue*10.0; // there was almost no effect with 0.1 (and 1000 as parameter)
// 	float h_kp = myPar.pid_h_p.sValue*0.01;
// 	float h_ki = myPar.pid_h_i.sValue*0.0001;
// 	float h_kd = myPar.pid_h_d.sValue*0.01;
// 
// 	float nav_max_accelint = myPar.nav_max_accelint.sValue*0.1; // m/s^2
// 	float nav_max_accel = myPar.nav_max_acc_lim.sValue*0.1; // m/s^2
// 	
// 	accel_command.x = PIDf(&pid_nav_x,speed_act_mps->x,setSpeed.x, nav_kp, nav_ki, nav_kd, -nav_max_accelint, nav_max_accelint); // tested
// 	accel_command.y = PIDf(&pid_nav_y,speed_act_mps->y,setSpeed.y, nav_kp, nav_ki, nav_kd, -nav_max_accelint, nav_max_accelint);
// 	accel_command.z = PIDf(&pid_nav_z,speed_act_mps->z,setSpeed.z, h_kp,   h_ki,   h_kd,   -nav_max_accelint, nav_max_accelint);
// 	
// 	accel_command_lim.x = limitf(accel_command.x, -nav_max_accel, nav_max_accel); // tested , works only with volatile variables on AVR32
// 	accel_command_lim.y = limitf(accel_command.y, -nav_max_accel, nav_max_accel);
// 	accel_command_lim.z = limitf(accel_command.z, -nav_max_accel, nav_max_accel);
// 	
// 	return accel_command_lim;
// }

// distance related governor
vector3_t NAV_Governor( vector3_t* pos_act_m, vector3_t* target_m ) 	
{
	volatile vector3_t accel_command;
	volatile vector3_t accel_command_lim;
	
	float nav_kp = myPar.pid_nav_p.sValue*0.01;
	float nav_ki = myPar.pid_nav_i.sValue*0.0001;
	float nav_kd = myPar.pid_nav_d.sValue*10.0;
	float h_kp = myPar.pid_h_p.sValue*0.01;
	float h_ki = myPar.pid_h_i.sValue*0.0001;
	float h_kd = myPar.pid_h_d.sValue*1.0;

	float nav_max_accelint = myPar.nav_max_accelint.sValue*0.1; // m/s^2
	float nav_max_accel = myPar.nav_max_acc_lim.sValue*0.1; // m/s^2
	
	accel_command.x = PIDf(&pid_nav_x,pos_act_m->x, target_m->x, nav_kp, nav_ki, nav_kd, -nav_max_accelint, nav_max_accelint);
	accel_command.y = PIDf(&pid_nav_y,pos_act_m->y, target_m->y, nav_kp, nav_ki, nav_kd, -nav_max_accelint, nav_max_accelint);

	accel_command.z = PIDf(&pid_nav_z,pos_act_m->z, target_m->z, h_kp,   h_ki,   h_kd,   -nav_max_accelint, nav_max_accelint);
	
	debug_gov.x = accel_command.x;
	debug_gov.y = pid_nav_x.I;
	debug_gov.z = pid_nav_x.old_diff;
	
	accel_command_lim.x = limitf(accel_command.x, -nav_max_accel, nav_max_accel); // tested , works only with volatile variables on AVR32
	accel_command_lim.y = limitf(accel_command.y, -nav_max_accel, nav_max_accel);
	accel_command_lim.z = limitf(accel_command.z, -nav_max_accel, nav_max_accel);
	
	return accel_command_lim;
}



void NAV_ResetPID(void)
{
	pid_nav_x.I = 0; pid_nav_x.old_diff = 0;
	pid_nav_y.I = 0; pid_nav_y.old_diff = 0;
	pid_nav_z.I = 0; pid_nav_z.old_diff = 0;
}

float debug_atot;

/*
Convert the desired acceleration into bank angle
all in global coordinates
acc_cmd: the desired acceleration [m/(s^2)] without earth acceleration (works differently on mars)
bank: the bank angle result in rad. 
This is the ROATATION AROUND the axis.
- Rotate around  y to move into x (lon) direction
- Rotate around -x to move into y (lat) direction
thrust: the thrust factor resulting from banking
*/
void GetBankAndThrustFromAccel(vector3_t acc_cmd, vector2_t* bank, float* thrust)
{
	//pay attention to gravity.
	//handle acc as vector. bank is the angle in x and y, thrust is the length.
	vector3_t acc;
	
	acc.x = 0;
	acc.y = 0;
	acc.z = 9.81; // earths acceleration (down) Just to stay flying.
	
	acc = vector_add(&acc,&acc_cmd); // add vectors

	// vector-calculation: bank x,y is the deviation of the resulting vector from normal.
	bank->y = atan2(acc.x,acc.z);
	bank->x = -atan2(acc.y,acc.z);
	
	float len = vector_len(&acc);
	
	if(len <= 0)
	{
		bank->y=0;
		bank->x=0;
		*thrust = 1;
		return;
	}
	
	// total angle out of z and vector length
	float angle_total = acos(acc.z/len);
	
	float MAX_lean_angle_rad = myPar.nav_max_angle.sValue * 0.01745329; // degrees to rad
		
	// reduce bank angle, if maximum is exceeded.
	debug_atot = angle_total;
	if(fabs(angle_total) > MAX_lean_angle_rad) 
	{
		float red = MAX_lean_angle_rad / fabs(angle_total);
		
		acc.x *= red; // reduce proportionally
		acc.y *= red;
		
		bank->y = limitf(atan2(acc.x,acc.z),-MAX_lean_angle_rad,MAX_lean_angle_rad);
		bank->x = -limitf(atan2(acc.y,acc.z),-MAX_lean_angle_rad,MAX_lean_angle_rad);
		
		len = vector_len(&acc);
	}
	
	if(acc.z <= 0) 
	{
		*thrust = 0.3; // todo taugt nix, vollgas und umdrehen?
		bank->x = 0;
		bank->y = 0;
	}
	else
	{
		// calculate thrust factor from vector length.
		*thrust = len * 0.10193; //= len / 9.81;
	}
	
	
	// winkel weiter reduzieren, wenn max-power erreicht fixme
}

#if TEST_RUN == 1

void test_NAV_Governor(void)
{
	vector3_t gov_actSpeed_m = {0,0,0}; // x,y,z = E N U
	vector3_t gov_actpos_m = {0,0,0}; // x,y,z = E N U
	vector3_t gov_setpos_m = {0,0,0}; // x,y,z = E N U
	vector3_t gov_testres_m = {0,0,0}; // x,y,z = E N U
//	myPar.nav_set_speed.sValue = 5;
	myPar.nav_decel_radius.sValue = 5;
	myPar.nav_decel_radiush.sValue = 3;
//	myPar.nav_set_speedh.sValue = 3;

	myPar.pid_nav_p.sValue = 100; // P of 1
	myPar.pid_nav_i.sValue = 0;
	myPar.pid_nav_d.sValue = 0;
//	myPar.nav_max_accel.sValue = 4;

	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,0);
	assertfequal(gov_testres_m.y,0);
	assertfequal(gov_testres_m.z,0);

	//x
	gov_setpos_m.x = 100;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,4);
	assertfequal(gov_testres_m.y,0);
	assertfequal(gov_testres_m.z,0);
	gov_setpos_m.x = -100;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,-4);
	assertfequal(gov_testres_m.y,0);
	assertfequal(gov_testres_m.z,0);
	
	//y
	gov_setpos_m.x = 0;
	gov_setpos_m.y = 100;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,0);
	assertfequal(gov_testres_m.y,4);
	assertfequal(gov_testres_m.z,0);
	gov_setpos_m.y = -100;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,0);
	assertfequal(gov_testres_m.y,-4);
	assertfequal(gov_testres_m.z,0);
	
	//z
	gov_setpos_m.y = 0;
	gov_setpos_m.z = 100;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,0);
	assertfequal(gov_testres_m.y,0);
	assertfequal(gov_testres_m.z,3);
	gov_setpos_m.z = -100;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,0);
	assertfequal(gov_testres_m.y,0);
	assertfequal(gov_testres_m.z,-3);
	gov_setpos_m.z = 0;
	
	// now with velocity...
	//x
	gov_actSpeed_m.x=10;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,-4);
	assertfequal(gov_testres_m.y,0);
	assertfequal(gov_testres_m.z,0);
	gov_actSpeed_m.x=-10;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,4);
	assertfequal(gov_testres_m.y,0);
	assertfequal(gov_testres_m.z,0);
	gov_actSpeed_m.x=0;
	
	//y
	gov_actSpeed_m.y=10;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,0);
	assertfequal(gov_testres_m.y,-4);
	assertfequal(gov_testres_m.z,0);
	gov_actSpeed_m.y=-10;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,0);
	assertfequal(gov_testres_m.y,4);
	assertfequal(gov_testres_m.z,0);
	gov_actSpeed_m.y=0;
	
	//z
	gov_actSpeed_m.z=10;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.y,0);
	assertfequal(gov_testres_m.x,0);
	assertfequal(gov_testres_m.z,-4);
	gov_actSpeed_m.z=-10;
	gov_testres_m = NAV_Governor(&gov_actpos_m,&gov_setpos_m);
	assertfequal(gov_testres_m.x,0);
	assertfequal(gov_testres_m.y,0);
	assertfequal(gov_testres_m.z,4);
}

void test_GetSetSpeed(void)
{
	/// GetSetSpeed
	
	myPar.nav_decel_radius.sValue = 10;	// The radius in meters, at which the cruise spee
	myPar.nav_decel_radiush.sValue = 5;	// The radius in meters, at which the height
	vector3_t actpos_m = {0,0,0}; // x,y,z = E N U
	vector3_t setpos_m = {20,0,10}; // x,y,z = E N U
	vector3_t desiredspeed_mps;

	// set pos variation

	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assert(fabs(desiredspeed_mps.x - 5.0) < epsilon);
	assert(fabs(desiredspeed_mps.y ) < epsilon);
	assert(fabs(desiredspeed_mps.z - 5.0) < epsilon);
	
	setpos_m.x = 5;
	setpos_m.y = 0;
	setpos_m.z = 2.5;
	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assert(fabs(desiredspeed_mps.x - 2.5) < epsilon);
	assert(fabs(desiredspeed_mps.y ) < epsilon);
	assert(fabs(desiredspeed_mps.z - 2.5) < epsilon);
	
	setpos_m.x = -5;
	setpos_m.y = 0;
	setpos_m.z = 2.5;
	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assert(fabs(desiredspeed_mps.x + 2.5) < epsilon);
	assert(fabs(desiredspeed_mps.y ) < epsilon);
	assert(fabs(desiredspeed_mps.z - 2.5) < epsilon);
	
	setpos_m.x = 0;
	setpos_m.y = 5;
	setpos_m.z = 2.5;
	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assert(fabs(desiredspeed_mps.x ) < epsilon);
	assert(fabs(desiredspeed_mps.y - 2.5) < epsilon);
	assert(fabs(desiredspeed_mps.z - 2.5) < epsilon);
	
	setpos_m.x = 0;
	setpos_m.y = -5;
	setpos_m.z = 2.5;
	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assert(fabs(desiredspeed_mps.x ) < epsilon);
	assert(fabs(desiredspeed_mps.y + 2.5) < epsilon);
	assert(fabs(desiredspeed_mps.z - 2.5) < epsilon);
	
	// act pos variation
	
	setpos_m.x = 0;
	setpos_m.y = 0;
	setpos_m.z = 0;
	
	actpos_m.x = 5;
	actpos_m.y = 0;
	actpos_m.z = 2.5;
	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assert(fabs(desiredspeed_mps.x + 2.5) < epsilon);
	assert(fabs(desiredspeed_mps.y ) < epsilon);
	assert(fabs(desiredspeed_mps.z + 2.5) < epsilon);
	
	actpos_m.x = -5;
	actpos_m.y = 0;
	actpos_m.z = 2.5;
	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assert(fabs(desiredspeed_mps.x - 2.5) < epsilon);
	assert(fabs(desiredspeed_mps.y ) < epsilon);
	assert(fabs(desiredspeed_mps.z + 2.5) < epsilon);
	
	actpos_m.x = 0;
	actpos_m.y = 5;
	actpos_m.z = 2.5;
	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assert(fabs(desiredspeed_mps.x ) < epsilon);
	assert(fabs(desiredspeed_mps.y + 2.5) < epsilon);
	assert(fabs(desiredspeed_mps.z + 2.5) < epsilon);
	
	actpos_m.x = 0;
	actpos_m.y = -5;
	actpos_m.z = 2.5;
	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assert(fabs(desiredspeed_mps.x ) < epsilon);
	assert(fabs(desiredspeed_mps.y - 2.5) < epsilon);
	assert(fabs(desiredspeed_mps.z + 2.5) < epsilon);
	
	
	
	actpos_m.x = -1.2;
	actpos_m.y = 3.4;
	actpos_m.z = 5.6;
	setpos_m.x = -1.2;
	setpos_m.y = 3.4;
	setpos_m.z = 5.6;

	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assertfequal(desiredspeed_mps.x,0);
	assertfequal(desiredspeed_mps.y,0);
	assertfequal(desiredspeed_mps.z,0);
	
	actpos_m.x = 0.01;  // if close to the target, the direction gets meaningless, which produces NaNs
	actpos_m.y = 0;
	actpos_m.z = 0;
	setpos_m.x = 0;
	setpos_m.y = 0;
	setpos_m.z = 0;

	desiredspeed_mps = GetSetSpeed(&actpos_m,&setpos_m);
	assertfequal(desiredspeed_mps.x,0);
	assertfequal(desiredspeed_mps.y,0);
	assertfequal(desiredspeed_mps.z,0);
}

void test_GetBankAndThrustFromAccel(void)
{
	/// GetBankAndThrustFromAccel
	
	vector2_t bank = {0,0};
	float thrust;
	vector3_t acc_cmd = {0,0,0};

	// x
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y - 0.0) < epsilon);
	assert(fabs(bank.x - 0.0) < epsilon);
	assert(fabs(thrust - 1.0) < epsilon);
	
	myPar.nav_max_angle.sValue = 90;	// The max lean angle in degrees (30)
	acc_cmd.x = 9.81;
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y - 0.78539816339744) < epsilon); // 45� expected  // rotation AROUND y !!
	assert(fabs(bank.x + 0.0) < epsilon);
	assert(fabs(thrust - sqrt(2.0)) < epsilon);
	
	acc_cmd.x = -9.81;
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y + 0.78539816339744) < epsilon); // 45� expected  // rotation AROUND y !!
	assert(fabs(bank.x + 0.0) < epsilon);
	assert(fabs(thrust - sqrt(2.0)) < epsilon);
	
	myPar.nav_max_angle.sValue = 30;	// The max lean angle in degrees (30)
	acc_cmd.x = 9.81;
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y - 0.523598775598298) < epsilon); // 30� expected
	assert(fabs(bank.x + 0.0) < epsilon);
	assert(fabs(thrust - 1.20177) < epsilon);
	
	acc_cmd.x = -9.81;
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y + 0.523598775598298) < epsilon); // 30� expected
	assert(fabs(bank.x + 0.0) < epsilon);
	assert(fabs(thrust - 1.20177) < epsilon);
	
	// y
	acc_cmd.x = 0;
	acc_cmd.y = 0;
	acc_cmd.z = 0;
	
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y - 0.0) < epsilon);
	assert(fabs(bank.x + 0.0) < epsilon);
	assert(fabs(thrust - 1.0) < epsilon);
	
	myPar.nav_max_angle.sValue = 90;	// The max lean angle in degrees (30)
	acc_cmd.y = 9.81;
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y - 0.0) < epsilon);
	assert(fabs(bank.x + 0.78539816339744) < epsilon); // 45� expected
	assert(fabs(thrust - sqrt(2.0)) < epsilon);

	acc_cmd.y = -9.81;
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y - 0.0) < epsilon);
	assert(fabs(bank.x - 0.78539816339744) < epsilon); // 45� expected
	assert(fabs(thrust - sqrt(2.0)) < epsilon);
	
	myPar.nav_max_angle.sValue = 30;	// The max lean angle in degrees (30)
	acc_cmd.y = 9.81;
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y - 0.0) < epsilon);
	assert(fabs(bank.x + 0.523598775598298) < epsilon); // 30� expected
	assert(fabs(thrust - 1.20177) < epsilon);
	
	acc_cmd.y = -9.81;
	GetBankAndThrustFromAccel(acc_cmd, &bank, &thrust);
	assert(fabs(bank.y - 0.0) < epsilon);
	assert(fabs(bank.x - 0.523598775598298) < epsilon); // 30� expected
	assert(fabs(thrust - 1.20177) < epsilon);
}





#endif // TEST_RUN


//// old stuff below



// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

/*
// 1st order CF
float Complementary(float newAngle, float newRate,int looptime) {
float tau=0.075;
float a=0.0;
float x_angleC;

	float dtC = float(looptime)/1000.0;
	a=tau/(tau+dtC);
	x_angleC= a* (x_angleC + newRate * dtC) + (1-a) * (newAngle);
	return x_angleC;
}


// 2nd order CF
float Complementary2(float newAngle, float newRate,int looptime) {
	float k=10;
	float dtc2=float(looptime)/1000.0;
	float x_angle2C,x1,x2;

	x1 = (newAngle -   x_angle2C)*k*k;
	y1 = dtc2*x1 + y1;
	x2 = y1 + (newAngle -   x_angle2C)*2*k + newRate;
	x_angle2C = dtc2*x2 + x_angle2C;


	return x_angle2C;
}

*/




