/*
 * SIM.c
 Simple simulation of position hold etc...
 Enables virtual flying (kind of)
 The orientation governor (gyro) is skipped here, this is for simulating the GPS-related stuff only.
 
 *
 * Created: 29.10.2014 22:39:56
 *
 * (c) 2014 by Fabian Huslik
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
#include "modules/vector.h"
#include "modules/quaternions/quaternions.h"
#include "modules/governing.h"
#include "SIM.h"

quaternion_t sim_orientation= {1,0,0,0};  
vector3_t sim_rate_radps= {0,0,0}; // rotational speed in rad/s
vector3_t sim_pos_m= {0,0,0}; // world position
vector3_t sim_vel_mps= {0,0,0}; // world velocity
float sim_accel_mpss = 0; // acceleration by thrust (filtered,therefore static)


quaternion_t SimGetorientation(void)
{
	return sim_orientation;
}

vector3_t SimGetRate(void)
{
	return sim_rate_radps;
}

vector3_t SimGetPos_m(void)
{
	return sim_pos_m;	
}

vector3_t SimGetVel_m(void)
{
	return sim_vel_mps;
}


float signf(float s)
{
	return (s<0.0f?-1.0f:1.0f);
}


// one simulation step
void SimDoLoop(int32_t ox, int32_t oy,int32_t oz, int32_t oa) // input the rotational command + thrust into simulation (values +- 4000 or 0..8000 for thrust)
{
	// calc acceleration caused by trust:
	float accel_mpss = oa;
	accel_mpss /= 8000.0;
	accel_mpss *= 9.81*2.5; // max acceleration is 2g
	Filter_f(sim_accel_mpss, accel_mpss, SIMPOWERFILTER);
	
	//create acceleration vector
	vector3_t vAccel_mpss;
	vAccel_mpss.x = 0;
	vAccel_mpss.y = 0;
	vAccel_mpss.z = accel_mpss;//make length = delta v
	vAccel_mpss = quaternion_rotateVector(vAccel_mpss,quaternion_inverse(sim_orientation)); // rotate into world orientation
	vAccel_mpss.z -=9.81; // subtract earths acceleration.
	
	//reduce the velocity by kind of air resistance
	// F = r * cw * A * v^2 /2
	// r = air density in kg/m³
	// A = area in m²
	
	// F = m*a
	// a = F / m

	vector3_t vAccAir_mpss;
	vAccAir_mpss.x = -signf(sim_vel_mps.x)*(SIMAIRDENSITY*SIMCWVALUE*SIMCOPTERAREA* (sim_vel_mps.x*sim_vel_mps.x)* 0.5) / SIMCOPTERMASS; // do not forget tho keep the sign, as it gets lost during squaring the speed.
	vAccAir_mpss.y = -signf(sim_vel_mps.y)*(SIMAIRDENSITY*SIMCWVALUE*SIMCOPTERAREA* (sim_vel_mps.y*sim_vel_mps.y)* 0.5) / SIMCOPTERMASS;
	vAccAir_mpss.z = -signf(sim_vel_mps.z)*(SIMAIRDENSITY*SIMCWVALUE*SIMCOPTERAREA* (sim_vel_mps.z*sim_vel_mps.z)* 0.5) / SIMCOPTERMASS; 
	
	vAccel_mpss = vector_add(&vAccel_mpss,&vAccAir_mpss); // add to total acceleration
	
	// add delta-v caused by acceleration to actual velocity reduce the velocity by kind of air resistance)
	vector3_t vDv = vector_scale(&vAccel_mpss,SIM_DT);
	
	sim_vel_mps = vector_add(&sim_vel_mps,&vDv);
	
	//add distance caused by v to actual pos
	vector3_t vTemp2 = vector_scale(&sim_vel_mps,SIM_DT);
	sim_pos_m = vector_add(&sim_pos_m, &vTemp2);
	
	// add influence of governor
	sim_rate_radps.x = Filter_f(sim_rate_radps.x,(float)ox*SIMRATEFACT,SIMRATEFLT); 
	sim_rate_radps.y = Filter_f(sim_rate_radps.y,(float)oy*SIMRATEFACT,SIMRATEFLT);
	sim_rate_radps.z = Filter_f(sim_rate_radps.z,(float)oz*SIMRATEFACT,SIMRATEFLT); // fixme different pars for yaw !!!
	
fixme
parameters too hefty (D needs to be reduced to 0)
	
	// rotate the actual rotation by a tiny amount
	quaternion_t qdiff;
	qdiff = quaternion_from_euler(sim_rate_radps.x*SIM_DT,sim_rate_radps.y*SIM_DT,sim_rate_radps.z*SIM_DT);
	sim_orientation = quaternion_multiply_flip_norm(sim_orientation,qdiff);
		
	
	if(sim_pos_m.z < 0.0)
	{
		SimReset();
	}
	
}


void SimReset(void) //reset the simulation to 0
{
	sim_orientation.w = 1;
	sim_orientation.x = 0;
	sim_orientation.y = 0;
	sim_orientation.z = 0;
	sim_rate_radps.x = 0;
	sim_rate_radps.y = 0;
	sim_rate_radps.z = 0;
	sim_pos_m.x= 0;
	sim_pos_m.y= 0;
	sim_pos_m.z= 0;	
	sim_vel_mps.x= 0;
	sim_vel_mps.y= 0;
	sim_vel_mps.z= 0;
	sim_accel_mpss = 0;

}