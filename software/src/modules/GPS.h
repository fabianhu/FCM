/*
 * GPS.h
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


#ifndef GPS_H_
#define GPS_H_

#define REG_MAP_SIZE       sizeof(gps_dataset)     //size of register map


typedef struct {
  uint8_t    new_data:1;
  uint8_t    gps2dfix:1;
  uint8_t    gps3dfix:1;
  uint8_t    wp_reached:1;
  uint8_t    numsats:4;
} GPS_STATUS_REGISTER;

// typedef struct {
//  uint8_t     poshold:1;
//  uint8_t     resume:1;
//  uint8_t     set_wp:1;
//  uint8_t     activate_wp:1;
//  uint8_t     wp:4;
// } GPS_COMMAND_REGISTER;

// typedef struct {
//  uint8_t     active_wp:4;
//  uint8_t     pervious_wp:4;
// } WAYPOINT_REGISTER;

typedef struct {
  int32_t      lat;            //degree*10 000 000
  int32_t      lon;            //degree*10 000 000
} gps_coordinates_t;

// typedef struct {
//   uint8_t   wp_reach_distance;		//If we are within this distance (in meters) then assumed that the waypoint is reached. Default value = 2m
//   uint8_t   reserved;				//reserved for future use
// } GPS_nav_par_t;

// typedef struct {
//   GPS_STATUS_REGISTER       status;            // 0x00  status register
//   GPS_COMMAND_REGISTER      command;           // 0x01  command register
//   WAYPOINT_REGISTER     wp_reg;            // 0x06  active waypoint and pervious waypoint (good for cross track error calculation)
//   uint16_t              ground_speed;      // 0x07-0x08 ground speed from gps km/h
//   int16_t               altitude;          // 0x09-0x0a gps altitude
//   uint32_t		time;	           // 0x0b-0x0e UTC Time from GPS
// 
//   uint16_t              distance;          // 0x0f-0x10 distance to active coordinates  (calculated)
//   int16_t               direction;         // 0x11-0x12 direction to active coordinates (calculated)   
//   gps_coordinates_t       gps_loc;           // 0x13 current location (8 byte)
//   gps_coordinates_t       gps_wp[16];         // 16 waypoints, WP#0 is RTH position
//   GPS_nav_par_t			wp_nav_par1;		//waypoint navigation parameter register 1
//   uint16_t				ground_course;		// 0x9c GPS ground cource
// } GPS_interfaceold_t;

typedef struct {
	float              ground_speed_mps;      // 0x07-0x08 ground speed from gps m/s
	float			   ground_course_rad;		// 0x9c GPS ground cource
	float              altitude;          // 0x09-0x0a gps altitude
	uint32_t		   time;	           // 0x0b-0x0e UTC Time from GPS
	GPS_STATUS_REGISTER       status;            // 0x00  status register
	gps_coordinates_t  gps_loc;           // 0x13 current location (8 byte)
} GPS_interface_t;

void gps_init(void);

uint32_t GPS_coord_to_degrees(char* s);
bool GPS_NMEA_newFrame(char c);

GPS_interface_t* gps_getDataPtr(void);

bool GPS_Start_Fix(void);
int GPSgetPos(gps_coordinates_t* pos);
uint8_t GPS_GetNumSats(void);
void GPS_timeout(void);

void GPS_distance_xy_m(gps_coordinates_t a, gps_coordinates_t b, volatile float* dist_x_lon, volatile float* dist_y_lat);
float GPS_calcHeading(float distx_m, float disty_m);
void test_GPS_calcHeading(void);
int GPSgetVelocity(float* pf_speed, float* pf_dir);

#endif /* GPS_H_ */