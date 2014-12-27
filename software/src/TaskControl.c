/*
 * TaskControl.c
 * This is where the magic happens (most of)
 *
 * Created: 06.09.2012 09:40:25
 *
 * (c) 2012-2014 by Fabian Huslik
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
#include "modules/types.h"
#include "modules/vector.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/spi.h"
#include "gyro.h"
#include "l3gd20.h"
#include "modules/servo_out.h"
#include "servo_in.h"
#include "UsartRC.h"
#include "menu/menu.h"
#include "menu/menu_variant.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "TaskControl.h"
#include "TaskLED.h"
#include "TaskTWI.h"
#include "modules/fcmcp.h"
#include "modules/quaternions/quaternions.h"
#include "modules/quaternions/MadgwickAHRS.h"
#include "modules/quaternions/MahonyAHRS.h"
#include "modules/quaternions/AHRS2.h" // in test !
#include "modules/emergency.h"
#include "modules/magnetics_calibration.h"
#include "modules/governing.h"
#include "modules/ParFlash.h"
#include "config.h"
#include "modules/vector.h"
#include "modules/GPS.h"
#include "modules/NAV.h"
#include "TaskNavi.h"
#include "modules/usart.h"
#include "modules/SIM.h"

void TaskControl(void);
void acc_calibrate(vector3_t* g);
void LoadParameters(void); 

fcm_data_t TXData; // data to be sent via FCMCP // todo maybe make a common buffer
fcm_quaternion_t TXQuaternions;
fcm_GPSdata_t TXGPSdata;

float debug_der_x; 
float debug_der_y;

IMUdata_t IMUdata; // info stuff

#define HEIGHTINITFILTER 10

uint32_t debug_RunTimeMax=0;
bool swAUX; // fixme global

/*

Servo input scaling:

throttle = 1000 = idle - 9000 full
roll: -4000 = right +4000 = left
elev : +4000 nose down -4000 nose up
yaw: -4000 = right +4000 = left

*/
int32_t plateau(int32_t pVal);
int32_t plateau(int32_t pVal)
{
	#define PLATEAUVAL 100
	
	if(abs(pVal)<PLATEAUVAL)
	{
		pVal = 0;
	}
	else if( pVal > 0)
		pVal = pVal-PLATEAUVAL;
	else
		pVal = pVal+PLATEAUVAL;
	return pVal;
}

void LoadParameters(void) 
{
	int16_t pardata_raw[MENUE_PARCOUNT];
	static int16_t* pdata;
	
	pdata = (int16_t*)(&myPar);
	
	int ret = ParFlash_load((uint8_t*)&pardata_raw, MENUE_PARCOUNT*2, PARAMETERVERSION); // we use them here, we load them here.
	
	if (ret == 0)
	{
		// load array of values only...
		for (int i = 0; i < MENUE_PARCOUNT;i++)
		{
			*pdata = pardata_raw[i];
			pdata += sizeof(Parameter_t)/2; // 2-byte-pointer
		}
	}
}

void TaskControl(void)
{
	static int32_t cmd_thr, cmd_roll, cmd_elev, cmd_yaw; // debug static
	
	bool swPosHold,swHGov,swRTH; // RC switch states
	int32_t pct; // fix name
	
	// measurement vectors
	vector3_t v_gyro_raw;
	vector3_t v_gyro_radps;
	vector3_t v_acc_frame_mpss; // includes gravity
	vector3_t v_acc_global_mpss; // without gravity, rotated into global frame
	vector3_t v_mag; 
	
	int32_t temperature_degC;

	quaternion_t q_ActualOrientation; // the actual rotation in global frame
	static quaternion_t q_RC_Set = {1,0,0,0}; // the official setpoint for the orientation.
	quaternion_t q_nav_global;
	quaternion_t q_set_global; // combined nav and rc

	float fActHeading_rad;
	
	vector3_t v_TrgDistance_m; // the distance to the target

	static float fSetHeading_rad = 0.0; // fixme also possible without this?? (one rc-input-quaternion, the rotation integration takes place by the frame itself)
	
	
	quaternion_t q_Diff;
	vector3_t v_diff_rad; // euler difference actual->set in rad

	int32_t ox, oy, oz, oa=0; // the output values for the output mixer
	static pid_t s_t_pid_x, s_t_pid_y, s_t_pid_z; // Gyro PID static values

	static vector3_t gyro_cal,acc_cal,mag_cal; // calibration values

	float dummy1,dummy2; // dummy results, needed as pointers

	//vector3_t v_RCCommand_rad;

	vector3_t res_cmd; // fixme change into rotation speed rad/s or so.

	int RCtimeout;
	
	static uint32_t RunTime; // run time measurement
	static uint32_t RunTimeStart=0; // run time measurement - remember the start time
	
	
	// globals from the best algo - ever
	vector3_t v_pos_act_m; // actual position offset from origin (global)
	vector3_t v_speed_act_mps; // actual speed in m/s (global)
	
	vector3_t v_pos_target_m;
	static vector3_t v_pos_last_m;
	
	float ax,ay; // fixme name
	vector3_t v_accel_command_mpss;
	vector2_t bank;
	
	
	LoadParameters();
	
	if(myPar.DirectMode.sValue != 0)
	{
		// wait for RX to get a signal before sending out the PWM (Servo signal)
		OS_WaitTicks(OSALM_CTRLWAIT,1000); 
	}
	
	acc_cal.x = (float)myPar.cal_acc_x.sValue/1000.0;
	acc_cal.y = (float)myPar.cal_acc_y.sValue/1000.0;
	acc_cal.z = (float)myPar.cal_acc_z.sValue/1000.0;
	gyro_cal.x = myPar.cal_gyro_x.sValue;
	gyro_cal.y = myPar.cal_gyro_y.sValue;
	gyro_cal.z = myPar.cal_gyro_z.sValue;
	mag_cal.x =  myPar.cal_mag_x.sValue;
	mag_cal.y =  myPar.cal_mag_y.sValue;
	mag_cal.z =  myPar.cal_mag_z.sValue;


	SPI_init();
	gyro_init();

	servo_in_init(OSALM_CTRLWAIT);
	servo_out_init(); // init outputs AFTER inputs, otherwise the valibration will not work.

	
	LED_SetFlightstate(FS_init);
	
	while(1)
	{
		wdt_clear(); // kick the dog
		
		RunTime = OS_GetTicks()-RunTimeStart; // "end" of runtime-measurement
		debug_RunTimeMax = max(debug_RunTimeMax,RunTime);
		OS_WaitEvent(OSEVT_CTLWAKEUP); //wait for isr, which synchronizes this task with the pwm output.
		RunTimeStart = OS_GetTicks(); // start of runtime-measurement
		
//		extern int debug_perlmutti; // fixme debug test only - we will need that again...
// 		static uint32_t lastpermut;
// 		if(OS_GetTicks()- lastpermut > 3000)
// 		{
// 			lastpermut = OS_GetTicks();
// 			debug_perlmutti++;
// 			if(debug_perlmutti >24) debug_perlmutti = 0;
// 		}
// 		int32_t svv;
// 		servo_in_get_ext_channel(5,&svv);
// 		debug_perlmutti = svv / 330;
		
		// Data acquisition
		RCtimeout = servo_in_getCmd(&cmd_thr, &cmd_roll, &cmd_elev, &cmd_yaw);
		swHGov = servo_in_get_ext_channel_switch(myPar.SwitchHGov.sValue,myPar.SwPHGov.sValue);
		swPosHold = servo_in_get_ext_channel_switch(myPar.SwitchPosHold.sValue,myPar.SwPPosHold.sValue);
		swRTH = servo_in_get_ext_channel_switch(myPar.SwitchReturnToHome.sValue,myPar.SwPReturnToHome.sValue);
		swAUX = servo_in_get_ext_channel_switch(myPar.SwitchAux.sValue,myPar.SwPAux.sValue);
		
		UpdatePotsFromTX(); // get poti values to parameters
		
		gyro_getValues_flt(&v_gyro_raw); // +- 10000 max

		v_gyro_raw = vector_subtract(&v_gyro_raw,&gyro_cal); // calibrate gyro values

		// convert gyros to rad/s
		v_gyro_radps.x = l3gd20_raw_to_rad(v_gyro_raw.x); // convert to rad / s
		v_gyro_radps.y = l3gd20_raw_to_rad(v_gyro_raw.y);
		v_gyro_radps.z = l3gd20_raw_to_rad(v_gyro_raw.z);

		vector3_t acc_raw;
		twi_getAccel_flt(&acc_raw); // fixme handle error return value !
		// resolution is 2 mg / bit -> 500 for 1 g -> 500 for 9.81 m/s^2 -> // measurement says 8000/g -> 8 bit / mg -> 1/8 mg / bit -> /8000 *9.81 -> 
		#define ACC_SCALE 0.00122625  // 0.01962 is wrong
		v_acc_frame_mpss = vector_scale(&acc_raw,ACC_SCALE); 
		
		v_acc_frame_mpss = vector_subtract(&v_acc_frame_mpss,&acc_cal);
		


		twi_getMagnet_flt(&v_mag);
		bool DoMagCal = myPar.cal_mag_auto_inflight.sValue ==1 ||  LED_GetFlightstate()==FS_idle; 
		v_mag = magcal_compensate(&v_mag,&mag_cal, DoMagCal); // hard iron error compensation; todo: save results
		
		temperature_degC = twi_get_temp();

		// at this point all sensors have been acquired.
		
		#if SIMULATION == 1
		// overwrite sensors by simulation
		v_gyro_radps = SimGetRate();
		v_gyro_raw = vector_scale(&v_gyro_radps,818.5); // reverse of the l3gd20_raw_to_rad // fixe check, if algo is possible without raw value.
		v_mag = SimGetMag();
		v_acc_frame_mpss = SimGetAcc();
		
		#endif

		// calculate actual attitude estimation
		switch(myPar.madgwick.sValue)
		{
			case 0:
				q_ActualOrientation = MahonyAHRSupdate(v_gyro_radps.x,v_gyro_radps.y,v_gyro_radps.z,v_acc_frame_mpss.x,v_acc_frame_mpss.y,v_acc_frame_mpss.z,v_mag.x,v_mag.y,v_mag.z); // todo make vector interface
				break;
			case 1:
				q_ActualOrientation = MadgwickAHRSupdate(v_gyro_radps.x,v_gyro_radps.y,v_gyro_radps.z,v_acc_frame_mpss.x,v_acc_frame_mpss.y,v_acc_frame_mpss.z,v_mag.x,v_mag.y,v_mag.z); // todo make vector interface
				break;
			case 2:
				q_ActualOrientation = AHRS2(v_gyro_radps,v_acc_frame_mpss,v_mag);
				break;
			default:
				emstop(77);
		}
		
		
		// calculate normalized acceleration	
		v_acc_global_mpss = quaternion_rotateVector(v_acc_frame_mpss,quaternion_inverse(q_ActualOrientation));
		v_acc_global_mpss.z -= 9.81; // subtract the z acceleration here.
		
		float filterAcc = myPar.nav_acc_flt_glob.sValue*0.001;
		
		static vector3_t v_accel_glob_flt_mpss;
		v_accel_glob_flt_mpss.x = Filter_f(v_accel_glob_flt_mpss.x,v_acc_global_mpss.x,filterAcc); // fixme this filter makes all worse! use BrownLinearExpo ??
		v_accel_glob_flt_mpss.y = Filter_f(v_accel_glob_flt_mpss.y,v_acc_global_mpss.y,filterAcc);
		v_accel_glob_flt_mpss.z = Filter_f(v_accel_glob_flt_mpss.z,v_acc_global_mpss.z,filterAcc);
		
		// always calculate the interpolation 			// The best of all algorithms - ever:
		Superfilter(v_accel_glob_flt_mpss, &v_pos_act_m, &v_speed_act_mps);  // internally disabled for now!

		quaternion_to_euler(q_ActualOrientation, &ax, &ay, &fActHeading_rad); // remember the actual heading
		
		// for the other systems:
		IMUdata.pitch_deg = ax*57.295779f; // todo is actually wrong, because global!!
		IMUdata.roll_deg = ay*57.295779f;
		IMUdata.mag_heading_deg = fActHeading_rad*57.295779f;
		IMUdata.height_dm = v_pos_act_m.z *100; // info
		
		float thrustfactor=1;
		extern int32_t debug_TWI_CountOfMisReads;  // dirty hack to keep flying, even if TWI crashes (which it usually only does during debugging)
		
		// get automatic flying setpoint
		if((swRTH || swPosHold || RCtimeout) && (debug_TWI_CountOfMisReads==0 || SIMULATION == 1))
		{	// quaternion based flying in auto level mode
			
			float fTrgNavDistance_m;
			float fTrgNavHeading_rad;
			float fHeadingDiff_rad;

			if(swPosHold)
			{
				// calculate command for returning to last position
				v_pos_target_m = vector_copy(&v_pos_last_m);
			}
			else // we return home anyway
			{
				v_pos_target_m.x = 0;
				v_pos_target_m.y = 0;
				// set trg h to last until we are near!!
				if(vector2len(v_pos_act_m.x,v_pos_act_m.y) > (float)myPar.nav_decel_radius.sValue)
				{
					v_pos_target_m.z = max(v_pos_last_m.z,2.0f);
					v_pos_last_m  = vector_copy(&v_pos_act_m);
					v_pos_last_m.z = v_pos_target_m.z; // preserve "last"
				}
				else
				{
					v_pos_target_m.z = 2.0f; // 2 m above ground, that it is still reachable
					v_pos_last_m  = vector_copy(&v_pos_act_m);
				}
				
			}
		
			// remember for waypoint flight: v_target_m = NAV_ConvertGPS_to_m(gpspos_setpoint,5); // calculate the setpoint in meter coordinates
				
			v_TrgDistance_m = vector_subtract(&v_pos_target_m,&v_pos_act_m); // get the 3D-vector to the target
			
			fTrgNavDistance_m = vector2len(v_TrgDistance_m.x,v_TrgDistance_m.y); // distance over ground
			if(fTrgNavDistance_m > 0.1)
			{
				fTrgNavHeading_rad = GPS_calcHeading(v_TrgDistance_m.x,v_TrgDistance_m.y); // target heading over ground
			
			} else
			{
				fTrgNavHeading_rad = fActHeading_rad;
			}
				
			// Get vehicle heading to calculate the vector.
			if(fTrgNavDistance_m > NAV_ROTATTETOTARGET_m && (swRTH || swPosHold) )
			{
				// rotate into heading, if more than 10 m away
				fHeadingDiff_rad = rectify_rad(fActHeading_rad-fTrgNavHeading_rad);
			}
			else
			{
				fHeadingDiff_rad = 0;
			}
				
			v_accel_command_mpss = NAV_Governor(&v_pos_act_m, &v_pos_target_m, &v_speed_act_mps); // inner parts tested, not completely!

			if(!swHGov && !RCtimeout)
			{
				v_accel_command_mpss.z = 0; // overwrite Z command to zero, if h governor is off.
			}
			
			if(!NAV_GPS_OK())
			{
				v_accel_command_mpss.x = 0; // overwrite command to zero, if pos governor is not possible.
				v_accel_command_mpss.y = 0;
			}
				
			GetBankAndThrustFromAccel(v_accel_command_mpss, &bank, &thrustfactor); // tested

			// obtain rotation offset by RC command input
			fSetHeading_rad += (float)cmd_yaw*0.00001; // magic number (try)
		
			// add rotation to target
			#if SIMULATION != 1 // do not rotate in simulation
				fSetHeading_rad += fHeadingDiff_rad*0.1; // fixme limit to a certain rate!
			#endif
		
			// limit rotation
			float fDiffheading_rad = rectify_rad(fActHeading_rad - fSetHeading_rad);

			#define ANGLE30DEG_rad 0.5235987
			#define ANGLE15DEG_rad 0.2617993
			if(fDiffheading_rad > ANGLE15DEG_rad) // limit the instant heading change to 15°
			{
				fSetHeading_rad = fActHeading_rad - ANGLE15DEG_rad;
			}
			if(fDiffheading_rad < -ANGLE15DEG_rad)
			{
				fSetHeading_rad = fActHeading_rad + ANGLE15DEG_rad;
			}
		
			fSetHeading_rad = rectify_rad(fSetHeading_rad);
		
			q_RC_Set = RC_Get_Offset(cmd_elev,-cmd_roll,fSetHeading_rad);

			quaternion_normalize(&q_RC_Set);
		
			q_nav_global = quaternion_from_euler(bank.x,bank.y,0); 

			q_set_global = quaternion_multiply_flip_norm(q_nav_global,q_RC_Set);


 			q_Diff = quaternion_multiply_flip_norm(quaternion_inverse(q_ActualOrientation),q_set_global); // calc difference in global orientation
		
			// transition to euler
			quaternion_to_euler(q_Diff,&v_diff_rad.x,&v_diff_rad.y,&v_diff_rad.z); // this is the difference in local orientation, because the global was inverse-multiplied (= "difference")

			
			#define NAV_MAX_CONTROL 2000 // acro mode scale is max 4000 // todo convert to rad/s
			
			// overwrite the rotational commands with quaternion results
			res_cmd.x = v_diff_rad.x * (float)myPar.quat_amp.sValue;
			res_cmd.x = limitf(res_cmd.x,-NAV_MAX_CONTROL,NAV_MAX_CONTROL);
			res_cmd.y = v_diff_rad.y * (float)myPar.quat_amp.sValue;
			res_cmd.y = limitf(res_cmd.y,-NAV_MAX_CONTROL,NAV_MAX_CONTROL);
			res_cmd.z = v_diff_rad.z * myPar.quat_amp.sValue/2;
			res_cmd.z = limitf(res_cmd.z,-NAV_MAX_CONTROL,NAV_MAX_CONTROL);	
			
		
		}
		else // acro mode
		{
			// normal flying with gyro PID
			quaternion_copy(&q_RC_Set,&q_ActualOrientation); // just to be prepared to proceed in another mode
			quaternion_to_euler(q_ActualOrientation, &dummy1, &dummy2, &fSetHeading_rad); // remember the actual heading
			
			NAV_ResetPID();			

			res_cmd.x = -plateau(cmd_elev)*myPar.Agilityfactor.sValue;
			res_cmd.y = -plateau(cmd_roll)*myPar.Agilityfactor.sValue;
			res_cmd.z = plateau(cmd_yaw)*myPar.Agilityfactor.sValue;
			
			//gps_coordinates_t gpspos_actual2;

			// if we do not hold position, we can remember the actual one as last pos.
			v_pos_last_m  = vector_copy(&v_pos_act_m);
			v_TrgDistance_m.x=0;v_TrgDistance_m.y=0;v_TrgDistance_m.z=0;
		}
		
		// governors
		pct = propscale(oa,0,10000,100,myPar.govAtMax.sValue); 
		
		// "classic" governor
		ox = PID(&s_t_pid_x, v_gyro_raw.x, res_cmd.x, myPar.pid_r_p.sValue*pct/100L, myPar.pid_r_i.sValue*pct/100L, myPar.pid_r_d.sValue*pct/100L, -myPar.integrator_limit.sValue, myPar.integrator_limit.sValue);
		oy = PID(&s_t_pid_y, v_gyro_raw.y, res_cmd.y, myPar.pid_r_p.sValue*pct/100L, myPar.pid_r_i.sValue*pct/100L, myPar.pid_r_d.sValue*pct/100L, -myPar.integrator_limit.sValue, myPar.integrator_limit.sValue);
		oz = PID(&s_t_pid_z, v_gyro_raw.z, res_cmd.z, myPar.pid_y_p.sValue*pct/100L, myPar.pid_y_i.sValue*pct/100L, myPar.pid_y_d.sValue*pct/100L, -myPar.integrator_limit.sValue, myPar.integrator_limit.sValue);

		
		float thr;
		thr = (float)cmd_thr*limit(thrustfactor,0.5,1.5); // factor is 1 in acro mode
			
			
		oa = limit((int32_t)thr,0,myPar.max_power.sValue); // always needed.

		

		if(cmd_thr < 1200) // set output to zero on stick max low.
		{
			// reset PID
			s_t_pid_x.I = 0; s_t_pid_x.old = 0;
			s_t_pid_y.I = 0; s_t_pid_y.old = 0;
			s_t_pid_z.I = 0; s_t_pid_z.old = 0;
			NAV_ResetPID();

			// copy actual to setpoint
			quaternion_copy(&q_RC_Set, &q_ActualOrientation);
			quaternion_to_euler(q_ActualOrientation, &dummy1, &dummy2, &fSetHeading_rad);
			ox=0;
			oy=0;
			oz=0;
			oa=0;
		}


		static int32_t heightflt = 0; // only for filtering the start height
		
	
		switch (LED_GetFlightstate())
		{
			case FS_init:
				//Parameters already loaded !
				
				NAV_SetOrigin_z_m((float) Filter_mem(&heightflt, twi_get_h_mm(),HEIGHTINITFILTER) * 0.001); // todo out of interpolated value!
				// remember GPS position as best idea...
				gps_coordinates_t gpspos_home_temp;
				if(GPSgetPos(&gpspos_home_temp)==0)
				{
					NAV_SetOrigin_xy_cm(gpspos_home_temp);
				}
				else
				{
					// todo set GPS to "failed"
				}
				
				if( MadgwickStart() && 
					servo_in_Start(cmd_thr, cmd_roll, cmd_elev, cmd_yaw)&& 
					!RCtimeout &&
					(GPS_Start_Fix() || myPar.waitGPSFix.sValue == 0)&&
					abs(IMUdata.pitch_deg) < STARTUP_MAX_ANGLE_DEV_DEG && abs(IMUdata.roll_deg) < STARTUP_MAX_ANGLE_DEV_DEG // todo naming is actually wrong, because global!!
					)
					{
						
						
						LED_SetFlightstate(FS_idle);
					}
					
				if(g_CalibrationRequest == 1) // sensor calibration required maybe no extra state, just do it here.
				{
					LED_SetFlightstate(FS_init_sensor_zero);
				}
					
				#if SKIPRCINIT == 1
				LED_SetFlightstate(FS_idle);
				#endif
	
				break;
			case FS_idle:
				if((cmd_thr < 1500) && (cmd_yaw < -3500) && (abs(cmd_elev) < 1000) && (abs(cmd_roll) < 1000))
				{
					if(LED_GetLastFlightstateChanged() > 100)
					{
						// remember start position
						gps_coordinates_t gpspos_home_temp;
						if(GPSgetPos(&gpspos_home_temp)==0)
						{
							NAV_SetOrigin_xy_cm(gpspos_home_temp);
						}
						else
						{
							// todo set GPS to "failed"
						}
						
						NAV_SetOrigin_z_m((float) Filter_mem(&heightflt, twi_get_h_mm(),HEIGHTINITFILTER) * 0.001); // todo out of interpolated value!
						
						// copy actual to setpoint
						quaternion_copy(&q_RC_Set, &q_ActualOrientation);

						quaternion_to_euler(q_ActualOrientation, &dummy1, &dummy2, &fSetHeading_rad);
						
						LED_SetFlightstate(FS_prefly);
					}
				}
				else
				{
					LED_SetFlightstate(FS_idle); // stay in this mode (reset timer)
					//startheight_mm = Filter_mem(&heightflt, height_baro_raw_mm,HEIGHTFILTER);
					
					if(g_CalibrationRequest == 1) // sensor calibration required maybe no extra state, just do it here.
					{
						LED_SetFlightstate(FS_init_sensor_zero);
					}
				}
				break;
			case FS_autoarm:
				// wait for zero g and go to fly then...
				if(0) // zeroG
				{
					// switch höhenregler on and add 5000mm to setpoint
					LED_SetFlightstate(FS_fly);
				}
				break;	
			case FS_prefly:
				// reset PID
				s_t_pid_x.I=0;s_t_pid_x.old=0;
				s_t_pid_y.I=0;s_t_pid_y.old=0;
				s_t_pid_z.I=0;s_t_pid_z.old=0;
				// copy actual to setpoint
				quaternion_copy(&q_RC_Set, &q_ActualOrientation);
				if(LED_GetLastFlightstateChanged() > 500)
				{
					LED_SetFlightstate(FS_fly);
				}			
				break;	
			case FS_fly:
			// ideas:
			//- start motors, one after another (maybe notify changes in r and a during startup and cancel, if too much.)
			//- calibrate zero of gyro again and verify with "no engine running" mode.(for 1 sec = 1000 measurements)
			//- switch motors off, if: az near 1, ax,ay near 0, rx,y,z near 0 AND user inputs throttle 0 and full yaw left ??
				if((cmd_thr < 1500) && (cmd_yaw > 3500) && (abs(cmd_elev) < 1000) && (abs(cmd_roll) < 1000))
				{
					if(LED_GetLastFlightstateChanged() > 100)
					{
						LED_SetFlightstate(FS_idle);
						// todo safe to flash of learned magnetometer values ?
					}
				}
				else
				{
					LED_SetFlightstate(FS_fly); // stay in this mode (reset timer)
				}
			
				break;
			case FS_init_sensor_zero:
			
				// todo: do restart, if bigger deviation (frame moved)
				gyro_calibrate(&gyro_cal);
				myPar.cal_gyro_x.sValue = gyro_cal.x;
				myPar.cal_gyro_y.sValue = gyro_cal.y;
				myPar.cal_gyro_z.sValue = gyro_cal.z;
				acc_calibrate(&acc_cal);
				myPar.cal_acc_x.sValue = acc_cal.x*1000;
				myPar.cal_acc_y.sValue = acc_cal.y*1000;
				myPar.cal_acc_z.sValue = acc_cal.z*1000;
				// fixme update the values
				mag_cal.x =0;
				mag_cal.y =0;
				mag_cal.z =0;
					
				g_CalibrationRequest = 0;
				LED_SetFlightstate(FS_init);
				break;				
			case FS_error:
				// we must NEVER come out of FS_fly ... otherwise we fall out of the sky.
				break;
			default:
				break;
		}
		
		
		// mag_calibration takes place in flight or during rotation, so rotate a lot.
		myPar.cal_mag_x.sValue = mag_cal.x;
		myPar.cal_mag_y.sValue = mag_cal.y;
		myPar.cal_mag_z.sValue = mag_cal.z;
		
		
		if(LED_GetFlightstate()== FS_fly)
		{
			#if SIMULATION == 0
			MixSetOut(ox, oy, oz, oa);
			#else
			SimDoLoop(ox, oy, oz, oa);
			#endif
		}
		else
		{
			#if SIMULATION == 1
			SimReset();
			#endif
			//  throttle cal only!!!
			if(myPar.DirectMode.sValue == 1)
			{
				servo_out_set(0,cmd_thr);
				servo_out_set(1,cmd_thr);
				servo_out_set(2,cmd_thr);
				servo_out_set(3,cmd_thr);
				servo_out_set(4,cmd_thr);
				servo_out_set(5,cmd_thr);
			}
			else if(myPar.DirectMode.sValue <0 && abs(IMUdata.pitch_deg) < 10 && abs(IMUdata.roll_deg) < 10 ) //direct mode with negative numbers runs individual motors, but only if angle <10° // todo naming is actually wrong, because global!!
			{
				for(int i=0;i<6;i++)
				{
					if(i+1 == -(myPar.DirectMode.sValue))
					servo_out_set(i,cmd_thr);
					else
					servo_out_set(i,0);
					
				}
			}
			else
			{
				servo_out_set(0,0);
				servo_out_set(1,0);
				servo_out_set(2,0);
				servo_out_set(3,0);
				servo_out_set(4,0);
				servo_out_set(5,0);
			}

		}
		
		#define radgra 57.295779513f
		static uint8_t fcmcp_TXdelayctr;
		
		if(fcmcp_TXdelayctr > 0)
		{
			fcmcp_TXdelayctr--;
		}
		else
		{
			fcmcp_TXdelayctr = fcmcp_getTXDelay();
					
			if (fcmcp_getStreamState() == fcmcp_stream_conv)
			{
				strncpy(TXData.hdr,"---D",4);
	
				extern uint8_t menu_show_diag;
	
				if(menu_show_diag)
				{
					// special data, if diag screen is on.
			//		extern vector3_t debug_accel,debug_gyro, debug_mag,debug_gov;
	
			// 		TXData.gx = debug_gyro.x*1000;//v_gyro_raw.x;
			// 		TXData.gy = debug_gyro.y*1000;//v_gyro_raw.y;
			// 		TXData.gz = debug_gyro.z*1000;//v_gyro_raw.z;
			// 		TXData.ax = debug_accel.x*1000; // 10000 = 1g
			// 		TXData.ay = debug_accel.y*1000;
			// 		TXData.az = debug_accel.z*1000;
			// 		TXData.mx = debug_mag.x*1000;// v_accel_command_mpss.x*1019; //v_mag.x;
			// 		TXData.my = debug_mag.y*1000;// v_accel_command_mpss.y*1019; //v_mag.y;
			// 		TXData.mz = debug_mag.z*1000;// v_accel_command_mpss.z*1019; //v_mag.z;
			// 		TXData.gov_x = debug_gov.x*1000;// v_accel_glob_flt_mpss.x*1019;//; //ox; // mag_cal.x;
			// 		TXData.gov_y = debug_gov.y*1000;// v_accel_glob_flt_mpss.y*1019;//0;// oy; // mag_cal.y;
			// 		TXData.gov_z = debug_gov.z*1000;// v_accel_glob_flt_mpss.z*1019;//0;// oz; // mag_cal.z;
			// 		TXData.RC_a = cmd_thr;
			// 		TXData.RC_x = cmd_elev;
			// 		TXData.RC_y = cmd_roll;
			// 		TXData.RC_z = cmd_yaw;
		
		
					TXData.gx =  debug_der_x*1000;//bank.x * radgra;//v_gyro_raw.x;
					TXData.gy = debug_der_y*1000;// bank.y * radgra;//v_gyro_raw.y;
					TXData.gz = fActHeading_rad*radgra;//thrust * 1000;//v_gyro_raw.z;
					TXData.ax = v_accel_command_mpss.x*1000;//fTrgNavDistance_m;
					TXData.ay = v_accel_command_mpss.y*1000;//fTrgNavHeading_rad* radgra;
					TXData.az = v_accel_command_mpss.z*1000;//fHeadingDiff_rad* radgra;
					TXData.mx = v_speed_act_mps.x*1000; //v_mag.x;
					TXData.my = v_speed_act_mps.y*1000; //v_mag.y;
					TXData.mz = v_speed_act_mps.z*1000; //v_mag.z;
					TXData.gov_x = v_pos_act_m.x*1000;//debug_gov.x*1000;// v_accel_glob_flt_mpss.x*1019;//; //ox; // mag_cal.x;
					TXData.gov_y = v_pos_act_m.y*1000;//debug_gov.y*1000;// v_accel_glob_flt_mpss.y*1019;//0;// oy; // mag_cal.y;
					TXData.gov_z = v_pos_act_m.z*1000;//debug_gov.z*1000;// v_accel_glob_flt_mpss.z*1019;//0;// oz; // mag_cal.z;
					TXData.RC_a = cmd_thr;
					TXData.RC_x = cmd_elev;
					TXData.RC_y = cmd_roll;
					TXData.RC_z = cmd_yaw;
				}
				else
				{
					// "normal" data
					TXData.gx = v_gyro_radps.x*radgra;
					TXData.gy = v_gyro_radps.y*radgra;
					TXData.gz = v_gyro_radps.z*radgra;
					TXData.ax = v_acc_frame_mpss.x*1000;
					TXData.ay = v_acc_frame_mpss.y*1000;
					TXData.az = v_acc_frame_mpss.z*1000;
					TXData.mx = v_mag.x;
					TXData.my = v_mag.y;
					TXData.mz = v_mag.z;
					TXData.gov_x = res_cmd.x;
					TXData.gov_y = res_cmd.y;
					TXData.gov_z = res_cmd.z;
					TXData.RC_a = cmd_thr;
					TXData.RC_x = cmd_elev;
					TXData.RC_y = cmd_roll;
					TXData.RC_z = cmd_yaw;
				}
	
	
				TXData.h = v_pos_act_m.z;
				TXData.temp[0] = twi_get_temp();
				strncpy(TXData.footer,"~~~",3);

				USART_Send(0,(uint8_t*)&TXData,sizeof(fcm_data_t));
			}else if  (fcmcp_getStreamState() == fcmcp_stream_quat)
			{
				strncpy(TXQuaternions.hdr,"---Q",4);

				TXQuaternions.qActAtt[0]= q_ActualOrientation.w;
				TXQuaternions.qActAtt[1]= q_ActualOrientation.x;
				TXQuaternions.qActAtt[2]= q_ActualOrientation.y;
				TXQuaternions.qActAtt[3]= q_ActualOrientation.z;
				
				/*float dx,dy,dz;
				quaternion_to_euler(q_ActualOrientation,&dx,&dy,&dz);
				quaternion_t qtest;
				qtest = quaternion_from_euler(dx,dy,dz);*/
				
				#if SIMULATION == 1

				quaternion_t q_simulated;
				q_simulated = SimGetorientation();				
				TXQuaternions.qSet[0]= q_set_global.w;
				TXQuaternions.qSet[1]= q_set_global.x;
				TXQuaternions.qSet[2]= q_set_global.y;
				TXQuaternions.qSet[3]= q_set_global.z;
				
				TXQuaternions.qSim[0]= q_simulated.w;
				TXQuaternions.qSim[1]= q_simulated.x;
				TXQuaternions.qSim[2]= q_simulated.y;
				TXQuaternions.qSim[3]= q_simulated.z;
				
				vector3_t v_simulated;
				v_simulated = SimGetPos_m();
				TXQuaternions.vPos[0] = v_simulated.x;
				TXQuaternions.vPos[1] = v_simulated.y;
				TXQuaternions.vPos[2] = v_simulated.z;
				
				#else
				TXQuaternions.qSet[0]= q_set_global.w;
				TXQuaternions.qSet[1]= q_set_global.x;
				TXQuaternions.qSet[2]= q_set_global.y;
				TXQuaternions.qSet[3]= q_set_global.z;
				
				TXQuaternions.qSim[0]= q_ActualOrientation.w;
				TXQuaternions.qSim[1]= q_ActualOrientation.x;
				TXQuaternions.qSim[2]= q_ActualOrientation.y;
				TXQuaternions.qSim[3]= q_ActualOrientation.z;
				
				TXQuaternions.vPos[0] = v_pos_act_m.x;
				TXQuaternions.vPos[1] = v_pos_act_m.y;
				TXQuaternions.vPos[2] = v_pos_act_m.z;
				#endif

				TXQuaternions.temp[1] = temperature_degC;
				
				strncpy(TXQuaternions.footer,"~~~",3);

				USART_Send(0,(uint8_t*)&TXQuaternions,sizeof(fcm_quaternion_t));
			}else if(fcmcp_getStreamState() == fcmcp_stream_GPS)
			{
				gps_coordinates_t txpos;
				GPSgetPos(&txpos);
			
			
				strncpy(TXGPSdata.hdr,"---G",4);
				TXGPSdata.lon = txpos.lon;
				TXGPSdata.lat = txpos.lat;
				TXGPSdata.hGPS = v_pos_act_m.z; // naaah duplicated.
				TXGPSdata.speedx = v_speed_act_mps.x;
				TXGPSdata.speedy = v_speed_act_mps.y;
				TXGPSdata.speedz = v_speed_act_mps.z;
				TXGPSdata.posx = v_pos_act_m.x;
				TXGPSdata.posy = v_pos_act_m.y;
				TXGPSdata.posz = v_pos_act_m.z;
				TXGPSdata.numsat = GPS_GetNumSats();
			
				strncpy(TXGPSdata.footer,"~~~",3);

				USART_Send(0,(uint8_t*)&TXGPSdata,sizeof(fcm_GPSdata_t));
			}
		}// delay counter

	} // while(1)
}


#define ACCCALFLT 0.01

// operates in m/s²
void acc_calibrate(vector3_t* cal) // todo : ausquartieren!
{
	vector3_t r;
	
	OS_WaitTicks(OSALM_CTRLWAIT,10);
	
	twi_getAccel_flt(&r);
	r = vector_scale(&r,ACC_SCALE); // scale to m/s²

	cal->x=r.x; // pre-load the filters with actual value
	cal->y=r.y;
	cal->z=r.z;
	
	for (uint32_t i = 0; i< 200;i++) // loop for 2s
	{
		
		OS_WaitTicks(OSALM_CTRLWAIT,10);
		
		if(twi_getAccel_flt(&r) != 0)
		{
			emstop(3);
		}
		r = vector_scale(&r,ACC_SCALE); // scale to m/s²
		
		cal->x = Filter_f(cal->x,r.x,ACCCALFLT);
		cal->y = Filter_f(cal->y,r.y,ACCCALFLT);
		cal->z = Filter_f(cal->z,r.z,ACCCALFLT);
	}
	
	cal->z -= 9.81;
	
}