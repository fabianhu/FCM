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

vector3_t sim_orient_rad;
float sim_power_mpss;

vector3_t sim_pos_m;
vector3_t sim_vel_mps;



// one simulation step
void SimStep10ms(void)
{
	
	
}

#define SIMBANKFILTER 0.1
#define SIMPOWERFILTER 0.3

// set the desired angles of orientation (before inputting to orientation governors, taken as the actual value here)
void SimSetBankRot_rad(vector3_t orientation_rad, float power_mpss)
{
	// filter the stuff
	sim_orient_rad.x = Filter_f(sim_orient_rad.x, orientation_rad.x, SIMBANKFILTER);
	sim_orient_rad.y = Filter_f(sim_orient_rad.y, orientation_rad.y, SIMBANKFILTER);
	sim_orient_rad.z = Filter_f(sim_orient_rad.z, orientation_rad.z, SIMBANKFILTER);
	sim_power_mpss = Filter_f(sim_power_mpss, power_mpss, SIMPOWERFILTER);
	
}


vector3_t SimGetPos_m(void)
{
	
}

vector3_t SimGetxxx(void)
{
	
}