/*
 * SIM.h
 *
 * Created: 29.10.2014 22:53:37
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


#ifndef SIM_H_ 
#define SIM_H_

#define SIM_DT 0.01 // fixed 10ms in our case
#define SIM_POWERFILTER 0.98 // for delaying the power application
#define SIM_RATEFLT 0.50 // delaying the application of the rate (simulated motor lag) todo experiment with it !
#define SIM_RATEFACT 0.0002 // factor by which the input rotation command is multiplied to get the rotation response from simulated copter todo experiment with it !

#define SIM_AIRDENSITY		1.1839	// in kg/m³
#define SIM_CWVALUE			1.1		// cw value without unit (for a cube)
#define SIM_COPTERAREA		0.05	// im m² 
#define SIM_COPTERAREATOP	0.2		// im m² 
#define SIM_COPTERMASS		0.8		// in kg

#define SIM_MAGFIELD_nT 60000.0f
#define SIM_MAGINC_DEG  60.0f
#define SIM_MAGDEFAULT_X 0.0f
#define SIM_MAGDEFAULT_Y cos(SIM_MAGINC_DEG/radgra)*SIM_MAGFIELD_nT
#define SIM_MAGDEFAULT_Z -sin(SIM_MAGINC_DEG/radgra)*SIM_MAGFIELD_nT

#define SIMFACTORLAT1 111.3 // gives m from degrees, correct only around "home"
#define SIMFACTORLON1 74.47 // gives m from degrees

#define SIMFACTORLAT 89.84 // gives 10 millionth degrees from m, correct only around "home"
#define SIMFACTORLON 134.27 // gives 10 millionth degrees from m

#define SIMGPSFILTER 0.15 // GPS delaying filter
#define SIMHFILTER   0.25 // height acquisition delaying filter 

#define SIMGPSDELAYBY 0 // was 4 ! fixme// delay the GPS signal by n calls (100 ms)

// visu only:
quaternion_t SimGetorientation(void); // get simulated orientation out of simulation
vector3_t SimGetPos_m(void); // get the simulated position out of the simulation

// simulated values:
vector3_t SimGetRate(void); // get simulated rotation rate (equivalent to gyro output)
vector3_t SimGetVel_m(void); // get the simulated velocity out of the simulation
vector3_t SimGetMag(void);
vector3_t SimGetAcc(void);

void SimDoLoop(int32_t ox, int32_t oy,int32_t oz, int32_t oa); // input the rotational command + thrust into simulation
void SimReset(void); //reset the simulation to 0
float signf(float s);

void SimDisturbStep(float* value, float* angle, float frequency_hz, float amplitude);
void SimDisturbVector(vector3_t *value, vector3_t *angle, float frequency_hz, float amplitude);

gps_coordinates_t SIM_GPS_getpos(void);
float SIM_geth(void);


#endif /* SIM_H_ */