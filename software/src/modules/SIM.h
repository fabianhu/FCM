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

#define SIM_DT 0.01
#define SIMPOWERFILTER 0.03
#define SIMROT 0.0001 // fixme experiment with it !

quaternion_t SimGetorientation(void); // get simulated orientation out of simulation
vector3_t SimGetPos_m(void); // get the simulated position out of the simulation
void SimDoLoop(int32_t ox, int32_t oy,int32_t oz, int32_t oa); // input the rotational command + thrust into simulation
void SimReset(void); //reset the simulation to 0

#endif /* SIM_H_ */