/*
 * SIM.h
 *
 * Created: 29.10.2014 22:53:37
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


#ifndef SIM_H_
#define SIM_H_

#define SIM_DT 0.01 // fixed 10ms in our case
#define SIMPOWERFILTER 0.98 // for delaying the power application
#define SIMRATEFLT 0.97 // fixme experiment with it !
#define SIMRATEFACT 0.0002 // factor by which the input rotation command is multiplied to get the rotation response from simulated copter fixme experiment with it !

#define SIMAIRDENSITY 1.1839	// in kg/m³
#define SIMCWVALUE    1.1		// cw value without unit
#define SIMCOPTERAREA 0.02		// im m² // 0.05*0.4
#define SIMCOPTERMASS 0.8		// in kg

quaternion_t SimGetorientation(void); // get simulated orientation out of simulation
vector3_t SimGetRate(void); // get simulated rotation rate (equivalent to gyro output)
vector3_t SimGetPos_m(void); // get the simulated position out of the simulation
vector3_t SimGetVel_m(void); // get the simulated velocity out of the simulation
void SimDoLoop(int32_t ox, int32_t oy,int32_t oz, int32_t oa); // input the rotational command + thrust into simulation
void SimReset(void); //reset the simulation to 0

#endif /* SIM_H_ */