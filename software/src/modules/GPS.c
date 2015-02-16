/*
 * GPS.c
 *
 * I2CGPS  - Inteligent GPS and NAV module for MultiWii by EOSBandi
 * V2.2   
 *
 * This program implements position hold and navigational functions for MultiWii by offloading caclulations and gps parsing 
 * into a secondary arduino processor connected to the MultiWii via i2c.
 * Once activated it outputs desired banking in a lat/lon coordinate system, which can be easily rotated into the copter's frame of reference.
 *
 * Navigation and Position hold routines and PI/PID libraries are based on code and ideas from the Arducopter team:
 * Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
 * Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni
 * Status blink code from Guru_florida
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */ 

#include <asf.h>
#include <fastmath.h>
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/vector.h"
#include "GPS.h"
#include "TaskNavi.h"
#include "config.h"
#include "testsuite.h"

//uint32_t GPS_distance_cm(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
//float GPS_bearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);

//void GPS_calc_longitude_scaling(int32_t lat);

#define sq(x) ((x)*(x))
#define isdigit(x) ((x)>47&&(x)<58)?1:0
#define DIGIT_TO_VAL(_x) (_x - '0')

GPS_interface_t   gps_dataset; 

// this is used to offset the shrinking longitude as we go towards the poles
//static float	GPS_scaleLonDown;
//static float	GPS_scaleLonUp;

/*void GPS_distance(gps_coordinates_t* src, gps_coordinates_t* dest, uint32_t* pdist_cm, float* pbearing_rad) 
{
	if(GPS_scaleLonUp == 0 || GPS_scaleLonDown == 0)
	{
		GPS_calc_longitude_scaling(src->lat);
	}
	
	*pdist_cm = GPS_distance_cm(src->lat,src->lon,dest->lat,dest->lon);
	*pbearing_rad = GPS_heading(*src,*dest); // invert to match FCM rotation scheme
}
*/



////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles	
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
// void GPS_calc_longitude_scaling(int32_t lat)
// {
// 	float rads 			= (abs((float)lat) / 10000000.0) * 0.0174532925; // in rad
// 	GPS_scaleLonDown 		= cos(rads);
// 	GPS_scaleLonUp 		        = 1.0f / cos(rads);
// }


////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
//
// float GPS_distance_cmx(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) 
// {
// 	if(GPS_scaleLonUp == 0 || GPS_scaleLonDown == 0)
// 	{
// 		GPS_calc_longitude_scaling(lat1);
// 	}  
// 	float dLat = (lat2 - lat1);                                    // difference of latitude in 1/10 000 000 degrees
// 	float dLon = (lon2 - lon1);
// 	dLon = dLon* GPS_scaleLonDown;
// 	float dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;
// 	return dist;
// }

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm x/y lat = NS lon = EW
//

// void GPS_distance_xy_cmx(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, float* dLat, float* dLon) 
// {
// 	if(GPS_scaleLonUp == 0 || GPS_scaleLonDown == 0)
// 	{
// 		GPS_calc_longitude_scaling(lat1);
// 	}
// 	
// 	float fdlat,fdlon;
// 	
// 	fdlat = (lat2 - lat1);
// 	fdlon = (lon2 - lon1);
// 		
// 	*dLat = fdlat * 1.113195;           //  1,5707963267948966192313216916398e-7 from earths circumference 40kkm                 // difference of latitude in 1/1 000 000 degrees
// 	*dLon = fdlon * GPS_scaleLonDown * 1.113195;
// }


// Circle assumption with earths circumference 40kkm         
void GPS_distance_xy_m(gps_coordinates_t a, gps_coordinates_t b, volatile float* dist_x_lon, volatile float* dist_y_lat)
{
	volatile float fdlat,fdlon,falat;
	
	fdlat = (b.lat - a.lat);
	fdlon = (b.lon - a.lon);
	falat = a.lat * 1.7453292519943295769236907684886e-9; // to rad from 10 000 000th deg
	
	*dist_y_lat = fdlat * 0.01113195 ;     // directly from earths circumference 40008km and 1/10 000 000 degrees
	*dist_x_lon = fdlon * cosf(falat) * 0.01113195;
}


/*
 	
 	
	(lat/lon in radians!)
	var R = 6371; // km
	var x = (lon2-lon1) * Math.cos((lat1+lat2)/2);
 	var y = (lat2-lat1);
 	var d = Math.sqrt(x*x + y*y) * R;


	virtual double distance(const Point & from, const Point & to) const {
		double lat1 = (from.lat() + to.lat()) / 2 * DEG_TO_RAD;
		double dx = 111.3 * cos(lat1) * (from.lon() - to.lon());
		double dy = 111.3 * (from.lat() - to.lat());
		return sqrt(dx * dx + dy * dy);
	};


*/

float GPS_calcHeading(float distx_m, float disty_m) 
{
	volatile float heading_rad;
	heading_rad = M_PI*2 - atan2f(disty_m,distx_m)+M_PI_2;
	if(heading_rad < 0) 
		heading_rad+=M_PI*2;
	if(heading_rad > M_PI*2) 
		heading_rad-=M_PI*2;
	
	return heading_rad;
}
#if TEST_RUN == 1
void test_GPS_calcHeading() 
{
		float angle;
	angle = GPS_calcHeading(5.0, 0);
	assertfequal(angle,M_PI_2);
	
	angle = GPS_calcHeading(-5.0, 0);
	assertfequal(angle,M_PI+M_PI_2);
	
	angle = GPS_calcHeading(0, 5.0);
	assertfequal(angle,0);
	
	angle = GPS_calcHeading(0, -5.0);
	assertfequal(angle,M_PI);
		
}
#endif

////////////////////////////////////////////////////////////////////////////////////
// get bearing from pos1 to pos2, returns rad
//
// float GPS_bearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
// {
// 	if(GPS_scaleLonUp == 0 || GPS_scaleLonDown == 0)
// 	{
// 		GPS_calc_longitude_scaling(lat1);
// 	}
// 
//     float off_x = (float)lon2 - lon1;
//     float off_y = ((float)(lat2 - lat1)) * GPS_scaleLonUp;
// 	float bearing = M_PI_2 + atan2(off_y, off_x) ; // add 90° because of "Einheitskreis" is defined 0° is right, not up.
// 
//     if (bearing < 0) bearing += M_PI*2; // ensure 0..2 pi
// 	return 2*M_PI-bearing; // rotate right = east.
// }



/* The latitude or longitude is coded this way in NMEA frames
dddmm.mmmm coded as degrees + minutes + minute decimal
This function converts this format in a unique unsigned long where 1 degree = 10 000 000
I increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
resolution also increased precision of nav calculations
*/

uint32_t GPS_coord_to_degrees(char* s)
{
	char *p, *q;
	uint8_t deg = 0, min = 0;
	unsigned int frac_min = 0;

	// scan for decimal point or end of field
	for (p = s; isdigit(*p); p++)
	;
	q = s;

	// convert degrees
	while ((p - q) > 2) {
		if (deg)
		deg *= 10;
		deg += DIGIT_TO_VAL(*q++);
	}
	// convert minutes
	while (p > q) {
		if (min)
		min *= 10;
		min += DIGIT_TO_VAL(*q++);
	}
	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.') {
		q = p + 1;
		for (int i = 0; i < 4; i++) {
			frac_min *= 10;
			if (isdigit(*q))
			frac_min += *q++ - '0';
		}
	}
	return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

/* This is am expandable implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA, GSA and RMC frames to decode on the serial bus
   Using the following data :
   GGA
     - time
     - latitude
     - longitude
     - GPS fix 
     - GPS num sat (5 is enough to be +/- reliable)
     - GPS alt
   GSA
     - 3D fix (it could be left out since we have a 3D fix if we have more than 4 sats  
   RMC
     - GPS speed over ground, it will be handy for wind compensation (future)  
     
*/

#define NO_FRAME    0
#define GPGGA_FRAME 1
#define GPGSA_FRAME 2
#define GPRMC_FRAME 3

bool GPS_NMEA_newFrame(char c) {

	uint8_t frameOK = 0;
	static uint8_t param = 0, offset = 0, parity = 0;
	static char string[15];
	static uint8_t checksum_param, gps_frame = NO_FRAME;

	switch (c) {
		case '$': param = 0; offset = 0; parity = 0;
		break;
		case ',':
		case '*': string[offset] = 0;
		if (param == 0) { //frame identification
			gps_frame = NO_FRAME;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') gps_frame = GPGGA_FRAME;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'S' && string[4] == 'A') gps_frame = GPGSA_FRAME;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') gps_frame = GPRMC_FRAME;
		}

		switch (gps_frame)
		{
			//************* GPGGA FRAME parsing
			case GPGGA_FRAME:
			switch (param)
			{
				case 1: gps_dataset.time = (atof(string)*1000); //up to .000 s precision not needed really but the data is there anyway
				break;
				//case 2: gps_dataset.gps_loc.lat = GPS_coord_to_degrees(string);
				case 2:  gps_dataset.gps_loc.lat = GPS_coord_to_degrees(string);
				break;
				//case 3: if (string[0] == 'S') gps_dataset.gps_loc.lat = -gps_dataset.gps_loc.lat;
				case 3: if (string[0] == 'S') gps_dataset.gps_loc.lat = -gps_dataset.gps_loc.lat;
				break;
				//case 4: gps_dataset.gps_loc.lon = GPS_coord_to_degrees(string);
				case 4:  gps_dataset.gps_loc.lon = GPS_coord_to_degrees(string);
				break;
				//case 5: if (string[0] == 'W') gps_dataset.gps_loc.lon = -gps_dataset.gps_loc.lon;
				case 5: if (string[0] == 'W') gps_dataset.gps_loc.lon = -gps_dataset.gps_loc.lon;
				break;
				case 6: gps_dataset.status.gps2dfix = string[0] > '0';
				break;
				case 7: gps_dataset.status.numsats = atoi(string);
				break;
				case 9: gps_dataset.altitude = atoi(string);
				break;
			}
			break;
			//************* GPGSA FRAME parsing
			case GPGSA_FRAME:
			switch (param)
			{
				case 2: gps_dataset.status.gps3dfix = string[0] == '3'; // fixme bitfield!!! one bit only!
				break;
			}
			break;
			//************* GPRMC FRAME parsing
			case GPRMC_FRAME:
			switch(param)
			{
				case 7: 
					//gps_dataset.ground_speed = (atof(string)*1.852); //convert knots to km/h (original value here was wrong)
					gps_dataset.ground_speed_mps = (atof(string)* 0.51444444444f); // convert knots to m/s
				break;
				case 8: 
					//gps_dataset.ground_course = (atof(string)*10); //Convert to degrees *10 (.1 precision)
					gps_dataset.ground_course_rad = (atof(string)*0.01745329251994329576923690768489f); // convert to rad
				break;
			}

			break;
		}

		param++; offset = 0;
		if (c == '*') checksum_param=1;
		else parity ^= c;
		break;
		case '\r':
		case '\n':
		if (checksum_param) { //parity checksum
			uint8_t checksum = 16 * ((string[0]>='A') ? string[0] - 'A'+10: string[0] - '0') + ((string[1]>='A') ? string[1] - 'A'+10: string[1]-'0');
			if (checksum == parity) frameOK = 1;
		}
		checksum_param=0;
		break;
		default:
		if (offset < 15) string[offset++] = c;
		if (!checksum_param) parity ^= c;

	}
	if( frameOK && (gps_frame == GPGGA_FRAME))
	{
		// remember time of frame
		// Navi_ReSetLastFrameTime();
		// notify Task
		OS_SetEventFromISR(OSTSK_NAVI,OSEVT_GPSREADY);
		return 1;
	}
	else
	return 0;
}



void gps_init() {

  uint8_t i;

  //Serial1.begin(115200);

  uint8_t *ptr = (uint8_t *)&gps_dataset;
  for (i=0;i<sizeof(gps_dataset);i++) { *ptr = 0; ptr++;}

  //Set up default parameters
  //gps_dataset.wp_nav_par1.wp_reach_distance = 2;			//If we are within 2 meters, consider the waypoint reached
    

}


/*GPS_interface_t* gps_getDataPtr(void)
{
	return &gps_dataset;
}*/

bool GPS_Start_Fix(void)
{
	if(
		gps_dataset.status.gps3dfix == 1 &&
		gps_dataset.status.numsats > 5
		)
	{
		return true;
	}
	else
	{
		return false;
	}
}


uint8_t GPS_GetNumSats(void)
{
	return gps_dataset.status.numsats;
}

void GPS_timeout(void) // timeout detected by external
{
	gps_dataset.status.numsats = 0;
	gps_dataset.status.gps2dfix = 0;
	gps_dataset.status.gps3dfix = 0;
}


int GPSgetPos(gps_coordinates_t* pos)
{
	if(gps_dataset.status.gps3dfix /*|| gps_dataset.status.gps2dfix*/ )
	{
		pos->lat = gps_dataset.gps_loc.lat;
		pos->lon = gps_dataset.gps_loc.lon;
		return 0;
	}
	else
	{
		// 
		return 1;
	}
}

int GPSgetVelocity(float* pf_speed, float* pf_dir)
{
	if(gps_dataset.status.gps3dfix /*|| gps_dataset.status.gps2dfix*/ )
	{
		*pf_speed = gps_dataset.ground_speed_mps;
		*pf_dir  = gps_dataset.ground_course_rad;
		return 0;
	}
	else
	{
		//
		return 1;
	}
}

/*
void GPS_Get_DistHdg(gps_coordinates_t a, gps_coordinates_t b, float* bearing, float* dist_x_lon, float* dist_y_lat)
{
	float dLon = b.lon - a.lon;
	*dist_y_lat = sinf(dLon) * cosf(b.lat);
	*dist_x_lon = cosf(a.lat)* sinf(b.lat) - sinf(a.lat)*cosf(b.lat)*cosf(dLon);
	float bear = atan2f(*dist_y_lat, *dist_x_lon);
	
	if(bear > 2*M_PI)
	{
		*bearing = bear-2*M_PI;
	}
	else if(bear < 0)
	{
		*bearing = bear+2*M_PI;
	}
	else
	{
		*bearing = bear;
	}
	
	
}
*/