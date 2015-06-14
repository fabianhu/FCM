/*
 * config.h
 *
 * Created: 15.08.2013 13:11:14
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


#ifndef CONFIG_H_
#define CONFIG_H_

// copter definitions for correct mix-matrix selection in servo_out.c
// HEXX
// QUADX
// ULICOPTER_HEX
// ULICOPTER_QUAD
#define QUADX

#define DISABLE_SENSOR_FUSION_GPS 1  // leave set to 1 until the sensor fusion is tested.

// run the test suite (DOES NOT FLY)
#define TEST_RUN 0
// skip the emergency stop conditions (just ignore missing accelerometer signal etc...)
#define NOEMSTOP 1  // DO NEVER SET TO 1 IF YOU INTEND TO LIVE.
// skip the RC init and set to SUMD, just to satisfy the init sequence.
#define SKIPRCINIT 0
// "fly" in simulated mode, all loops will be closed by simulation in SIM.c
#define SIMULATION 1 

//usart2 = lower = GPS
//usart0 = upper, 
// select only one!

#define COM0_FCMCP 1
#define COM0_MAVLINK 0
#define COM0_HOTT 0

// PPM signal is inverted for FUTABA -> 1 ; todo make auto.
#define PPMINVERTED 1 // fixme positive PPM does not work!

#define STARTUP_MAX_ANGLE_DEV_DEG 15 // maximal angle in degrees, which the FCM can be out of level for startup


// final checks

#if TEST_RUN + NOEMSTOP + SKIPRCINIT +SIMULATION != 0
	#warning DO NOT FLY
#endif

#if COM0_FCMCP + COM0_MAVLINK + COM0_HOTT != 1
	#error "Select one mode!"
#endif


#endif /* CONFIG_H_ */