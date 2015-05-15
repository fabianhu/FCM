/*
 * SIM.c
 Simple simulation of vehicle dynamics.
 Enables virtual flying.
 The orientation governor (gyro) is included here.
 
 
 *
 * Created: 29.10.2014 22:39:56
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

#define radgra 57.295779513f

#include <asf.h>
#include <fastmath.h>
#include "modules/vector.h"
#include "modules/quaternions/quaternions.h"
#include "modules/governing.h"
#include "../menu/menu.h"
#include "../menu/menu_variant.h"
#include "../FabOS_config.h"
#include "../FabOS/FabOS.h"
#include "GPS.h"
#include "SIM.h"

quaternion_t sim_orientation= {1,0,0,0};  
vector3_t sim_rate_radps= {0,0,0}; // rotational speed in rad/s
vector3_t sim_rate_radps_dist= {0,0,0}; // rotational speed in rad/s (disturbed)
vector3_t sim_pos_m= {0,0,0}; // world position
vector3_t sim_vel_world_mps= {0,0,0}; // world velocity
float sim_accel_mpss = 0; // acceleration by thrust (filtered,therefore static)
vector3_t sim_accel_frame_mpss = {0,0,-9.81f};
vector3_t sim_magneto_frame_nT = {0,0,0};

// for display purposes only, the flight controller should not know about the simulated orientation.
quaternion_t SimGetorientation(void)
{
	return sim_orientation;
}

// returns the rate as measured by a gyroscope
vector3_t SimGetRate(void)
{
	//vector3_t gyro_drift = {0.03,0.03,0.03};
	
	//return vector_add(&gyro_drift, &sim_rate_radps);
	return sim_rate_radps_dist;
}

// simulated position - for display purposes only, the flight controller should not know about the simulated position directly.
vector3_t SimGetPos_m(void)
{
	return sim_pos_m;
}

vector3_t SimGetVel_m(void)
{
	return sim_vel_world_mps;
}

// returns the flux as measured by a magnetometer
vector3_t SimGetMag(void)
{
	return sim_magneto_frame_nT;
}

// returns the acceleration as measured by a accelerometer
vector3_t SimGetAcc(void)
{
	return sim_accel_frame_mpss;
}


float signf(float s)
{
	return (s<0.0f?-1.0f:1.0f);
}

gps_coordinates_t SIM_GPS_getpos(void)
{
	static vector2_t filtered;
	
	static float posx[SIMGPSDELAYBY];
	static float posy[SIMGPSDELAYBY];
	
	static uint32_t wr = 0;
	static uint32_t rd = 1;
	
	vector3_t v_simul = SimGetPos_m();	
	posx[wr] = v_simul.x;
	posy[wr] = v_simul.y;

	gps_coordinates_t ActPosSim = {483829100 , 108527100}; // FCM "home"
	
	filtered.x = Filter_f(filtered.x,posx[rd],SIMGPSFILTER);
	filtered.y = Filter_f(filtered.y,posy[rd],SIMGPSFILTER);
	
	
	ActPosSim.lon += (int32_t)(filtered.x * SIMFACTORLON);
	ActPosSim.lat += (int32_t)(filtered.y * SIMFACTORLAT);
	
	// increase delay counters
	if (++wr == SIMGPSDELAYBY) wr = 0;
	if (++rd == SIMGPSDELAYBY) rd = 0;
	
	return ActPosSim;
}


float SIM_geth(void)
{
	static float filtered;
		
	vector3_t v_simul = SimGetPos_m();
	
	filtered = Filter_f(filtered,v_simul.z,SIMHFILTER);
	
	return filtered;
}


// one simulation step
void SimDoLoop(int32_t ox, int32_t oy,int32_t oz, int32_t o_thrust) // input the rotational command + thrust into simulation (values +- 4000 or 0..8000 for thrust)
{
	static vector3_t DisturbAngleGyro;
	static vector3_t DisturbAngleAcc;
	static vector3_t DisturbAngleMag;
	static vector3_t DisturbAngleWind;
	
	// calc acceleration caused by trust:
	float thrust_mpss = o_thrust;
	thrust_mpss /= 8000.0;
	thrust_mpss *= 9.81 * 2.3; // max acceleration is 2.3G
	Filter_f(sim_accel_mpss, thrust_mpss, SIM_POWERFILTER);
	
	vector3_t vel_vehicle;
	vel_vehicle = vector_copy(&sim_vel_world_mps); 
	
	SimDisturbStep(&vel_vehicle.x,&DisturbAngleWind.x,(float)myPar.wind_freq.sValue*0.1, (float)myPar.wind_ampl.sValue*0.1); // add wind	xy only
	SimDisturbStep(&vel_vehicle.y,&DisturbAngleWind.y,(float)myPar.wind_freq.sValue*0.1, (float)myPar.wind_ampl.sValue*0.1);
	
	vel_vehicle = quaternion_rotateVector(vel_vehicle,quaternion_inverse(sim_orientation)); // rotate into vehicle orientation // fixme this "inverse" seems not plausible, but works better than without.

	//create vehicle acceleration vector
	vector3_t vAccel_mpss;
	vAccel_mpss.x = 0;
	vAccel_mpss.y = 0;
	vAccel_mpss.z = thrust_mpss;
	
	//reduce the velocity by air resistance
	// F = r * cw * A * v^2 /2
	// r = air density in kg/m³
	// A = area in m²	
	// F = m*a
	// a = F / m
	vAccel_mpss.x -= signf(vel_vehicle.x) * (SIM_AIRDENSITY*SIM_CWVALUE*SIM_COPTERAREA* (vel_vehicle.x*vel_vehicle.x)* 0.5) / SIM_COPTERMASS; 
	vAccel_mpss.y -= signf(vel_vehicle.y) * (SIM_AIRDENSITY*SIM_CWVALUE*SIM_COPTERAREA* (vel_vehicle.y*vel_vehicle.y)* 0.5) / SIM_COPTERMASS;
	vAccel_mpss.z -= signf(vel_vehicle.z) * (SIM_AIRDENSITY*SIM_CWVALUE*SIM_COPTERAREATOP* (vel_vehicle.z*vel_vehicle.z)* 0.5) / SIM_COPTERMASS; 
	OS_DISABLEALLINTERRUPTS
	sim_accel_frame_mpss = vector_copy(&vAccel_mpss);// simulation output
	SimDisturbVector(&sim_accel_frame_mpss,&DisturbAngleAcc,(float)myPar.accel_freq.sValue,(float) myPar.accel_ampl.sValue*0.1); // add noise only for simulated measurement
	sim_accel_frame_mpss = vector_scale(&sim_accel_frame_mpss,-1); // the measured = resulting acceleration is the other way!!
	OS_ENABLEALLINTERRUPTS
	
	vAccel_mpss = quaternion_rotateVector(vAccel_mpss,sim_orientation); // rotate into world orientation
	
	vAccel_mpss.z -=9.81; // subtract earths acceleration.
	// in steady state, the earths acceleration and the "o_thrust" acceleration neutralize to 0.
	
	vector3_t vDv = vector_scale(&vAccel_mpss,SIM_DT); // multiply by time -> delta v for this step
	
	sim_vel_world_mps = vector_add(&sim_vel_world_mps,&vDv);
	
	//add distance caused by v to actual pos
	vector3_t vTemp2 = vector_scale(&sim_vel_world_mps,SIM_DT);
	sim_pos_m = vector_add(&sim_pos_m, &vTemp2);
	
	// add influence of governor
	sim_rate_radps.x = Filter_f(sim_rate_radps.x,(float)ox*SIM_RATEFACT,SIM_RATEFLT);
	sim_rate_radps.y = Filter_f(sim_rate_radps.y,(float)oy*SIM_RATEFACT,SIM_RATEFLT);
	sim_rate_radps.z = Filter_f(sim_rate_radps.z,(float)oz*SIM_RATEFACT,SIM_RATEFLT); // fixme different pars for yaw !!!

	OS_DISABLEALLINTERRUPTS
	sim_rate_radps_dist = vector_copy(&sim_rate_radps);
	SimDisturbVector(&sim_rate_radps_dist,&DisturbAngleGyro,(float)myPar.gyro_freq.sValue, (float)myPar.gyro_ampl.sValue*0.1f); // add noise
	OS_ENABLEALLINTERRUPTS

	// rotate the actual rotation by a tiny amount
	quaternion_t qdiff;
	qdiff = quaternion_from_euler(sim_rate_radps.x*SIM_DT,sim_rate_radps.y*SIM_DT,sim_rate_radps.z*SIM_DT);
	sim_orientation = quaternion_multiply_flip_norm(sim_orientation,qdiff);
	
	if(sim_pos_m.z <= 0.0)
	{
		SimReset();
	}
	
	vector3_t mag_world;
	
	mag_world.x = SIM_MAGDEFAULT_X;
	mag_world.y = SIM_MAGDEFAULT_Y;
	mag_world.z = SIM_MAGDEFAULT_Z;
	float rotation_disturbance = 0;
	static float rotation_disturbance_angle = 0;
	SimDisturbStep(&rotation_disturbance,&rotation_disturbance_angle,(float)myPar.mag_mis_freq.sValue*0.01f,(float)myPar.mag_mis_ampl.sValue*0.06283f); // max disturbance 2Pi
	
	quaternion_t rotation = quaternion_from_euler(0,0,rotation_disturbance);
	rotation = quaternion_multiply_flip_norm(sim_orientation,rotation);
	
	mag_world = quaternion_rotateVector(mag_world,quaternion_inverse(rotation)); // simulation output rotated into vehicle frame
	SimDisturbVector(&mag_world,&DisturbAngleMag, (float)myPar.magneto_freq.sValue*0.01, (float)myPar.magneto_ampl.sValue*1000.0f); // add noise
	sim_magneto_frame_nT = vector_copy( &mag_world);
}


void SimReset(void) //reset the simulation to 0
{
	sim_orientation.w = 1;
	sim_orientation.x = 0;
	sim_orientation.y = 0;
	sim_orientation.z = 0;

	sim_pos_m.x= 0;
	sim_pos_m.y= 0;
	sim_pos_m.z= 0;	
	sim_vel_world_mps.x= 0;
	sim_vel_world_mps.y= 0;
	sim_vel_world_mps.z= 0;
	sim_accel_mpss = 0;

	sim_rate_radps.x = 0;
	sim_rate_radps.y = 0;
	sim_rate_radps.z = 0;
	
	sim_accel_frame_mpss.x = 0;
	sim_accel_frame_mpss.y = 0;
	sim_accel_frame_mpss.z = -9.81f; // earths acceleration.
	
	sim_magneto_frame_nT.x = SIM_MAGDEFAULT_X;
	sim_magneto_frame_nT.y = SIM_MAGDEFAULT_Y;
	sim_magneto_frame_nT.z = SIM_MAGDEFAULT_Z;

}

void SimDisturbStep(float* value, float* angle, float frequency_hz, float amplitude)
{
	if(frequency_hz <=0.001)
	{
		*value+=amplitude;
		return;
	}
	
	*angle += (frequency_hz*SIM_DT)*(2*M_PI);
	while(*angle > 2*M_PI)
	{
		*angle -= 2*M_PI;
	}
	
	*value += (sinf(*angle)*amplitude);
}


void SimDisturbVector(vector3_t *value, vector3_t *angle, float frequency_hz, float amplitude)
{
	if(frequency_hz <=0.001)
	{
		value->x += amplitude;
		value->y += amplitude;
		value->z += amplitude;
		return;
	}
	
	angle->x += (frequency_hz*SIM_DT)*(2*M_PI);
	while(angle->x > 2*M_PI)
	{
		angle->x -= 2*M_PI;
	}	
	
	angle->y += (frequency_hz*0.95f*SIM_DT)*(2*M_PI) ;
	while(angle->y > 2*M_PI)
	{
		angle->y -= 2*M_PI;
	}
	
	angle->z += (frequency_hz*1.05f*SIM_DT)*(2*M_PI);
	while(angle->z > 2*M_PI)
	{
		angle->z -= 2*M_PI;
	}
	
	value->x += (sinf(angle->x)*amplitude);
	value->y += (sinf(angle->y)*amplitude);
	value->z += (sinf(angle->z)*amplitude);
}