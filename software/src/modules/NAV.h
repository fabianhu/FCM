/*
 * NAV.h
 *
 * Created: 18.10.2013 23:55:37
 *
 * (c) 2013-2015 by Fabian Huslik
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


#ifndef NAV_H_
#define NAV_H_

#define NAVGPSTIMEOUT_ms 500
#define NAV_ROTATTETOTARGET_m 10

// set origin (first lock / arm)
void NAV_SetOrigin_xy_cm(gps_coordinates_t pos);
void NAV_SetOrigin_z_m(float h);

// Convert a GPS position relative to the origin, which has been set with NAV_SetOrigin_xy_cm.
vector2_t NAV_ConvertGPS_to_m(gps_coordinates_t set);

// update position from GPS handler
void NAV_UpdatePosition_xy(gps_coordinates_t coords);

// update height from barometer handler
void NAV_UpdatePosition_z_m(float h);

// check, if GPS was updated recently
bool NAV_GPS_OK(void);

/*
Filter (slow) position and (fast) accelometer into position and velocity.
All in global coordinates.
acc_mpss: Accerleration world based without gravity in meters / s*s
pos: gps position and barometer height
*/
void Superfilter(vector3_t acc_mpss, vector3_t* pos_act);

/*
Get the desired speed setpoint out of the distance to trg.
out: speed setpoint in m/s
*/
vector3_t GetSetSpeed(vector3_t* actPos_m, vector3_t* setPos_m);
void test_GetSetSpeed(void);

/*
The Governor
return the desired acceleration vector
*/
vector3_t NAV_Governor( vector3_t* pos_act_m, vector3_t* target_m );
void test_NAV_Governor(void);

/*
Reset the PIDs to 0
*/
void NAV_ResetPID(void);

/*
pay attention to gravity.
handle acc as vector. bank is the angle in x and y, thrust is the length.
*/
void GetBankAndThrustFromAccel(vector3_t acc_cmd, vector2_t* bank, float* thrust);
void test_GetBankAndThrustFromAccel(void);

#endif /* NAV_H_ */