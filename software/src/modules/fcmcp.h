/*
 * fcmcp.h FCM Communication Protocol
 *
 * Created: 28.01.2013 22:08:33
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


#ifndef FCMCP_H_
#define FCMCP_H_

typedef enum
{
	fcmcp_stream_idle=0,
	fcmcp_stream_conv,
	fcmcp_stream_quat,
	fcmcp_stream_GPS,
	fcmcp_stream_menu,
	fcmcp_stream_parl
}fcmcp_state_t;

typedef struct fcm_data_tag 
{
	char hdr[4];					// ---D
	int32_t gx,gy,gz;				// Gyro
	int32_t ax,ay,az;				// Accelerometer
	int32_t mx,my,mz;				// Magneto
	int32_t gov_x,gov_y,gov_z;		// Governor out
	int32_t RC_x,RC_y,RC_z,RC_a;	// RC command
	int32_t h;						// height in mm (barometric measurement)
	int16_t temp[2];				// Temperature in 10th °C
	char footer[4];					// ~~~0
}fcm_data_t;

typedef struct fcm_quaternion_tag
{
	char hdr[4];					// ---Q
	float qActAtt[4];				// measured / estimated attitude
	float qSet[4];					// setpoint attitude
	float qSim[4];					// simulated attitude (SIMULATION only)
	float vPos[3];					// World position
	float vDat[3];					// extra data
	/*int32_t h;						// height in mm (barometric measurement)
	int16_t temp[2];				// Temperature in 10th °C*/
	char footer[4];					// ~~~0
}fcm_quaternion_t;

typedef struct fcm_GPSdata_tag
{
	char hdr[4];					// ---G
	int32_t lon;					// longitude in 10.000.000th degrees
	int32_t lat;					// latitude in 10.000.000th degrees
	int32_t hGPS;					// gps height in full m
	float speedx;					// x (lon) speed [m/s]
	float speedy;					// y (lat) speed [m/s]
	float speedz;					// z (up)  speed [m/s]
	float posx;						// x (lon) distance [m]
	float posy;						// y (lat) distance [m]
	float posz;						// z (up) distance [m]
	uint8_t numsat;					// number of satellites
	char footer[4];					// ~~~0
}fcm_GPSdata_t;


void fcmcp_analyzePacket(char* buf);
fcmcp_state_t fcmcp_getStreamState(void);
uint8_t fcmcp_getTXDelay(void);
void fcmcp_setStreamState(fcmcp_state_t e);

#endif /* FCMCP_H_ */