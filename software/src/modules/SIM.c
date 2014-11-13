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
vector3_t sim_pos_m= {0,0,0};
vector3_t sim_vel_mps= {0,0,0};
float sim_accel_mpss;
vector3_t sim_rotationalspeed_radps = {0,0,0};



quaternion_t SimGetorientation(void)
{
	return sim_orientation;
}

vector3_t SimGetPos_m(void)
{
	return sim_pos_m;	
}

// one simulation step
void SimDoLoop(int32_t ox, int32_t oy,int32_t oz, int32_t oa) // input the rotational command + thrust into simulation (values +- 4000 or 0..8000 for thrust)
{
	// calc acceleration caused by trust:
	float accel_mpss = oa;
	accel_mpss /= 8000.0;
	accel_mpss *= 9.81*2.0; // max acceleration is 2g
	Filter_f(sim_accel_mpss, accel_mpss, SIMPOWERFILTER);
	
	//create accelleration vector
	vector3_t v_accel_mpss;
	v_accel_mpss.x = 0;
	v_accel_mpss.y = 0;
	v_accel_mpss.z = accel_mpss;//make length = delta v
	v_accel_mpss = quaternion_rotateVector(v_accel_mpss,sim_orientation); // rotate into orientation
	
	//add distance by v to actual pos
	sim_pos_m = vector_add(sim_pos_m, vector_scale(sim_vel_mps,SIM_DT));
	
	
	
	quaternion_t qdiff;
	quaternion_from_euler()
	
	
}


void SimReset(void) //reset the simulation to 0
{
	
}