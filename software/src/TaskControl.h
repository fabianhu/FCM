/*
 * TaskControl.h
 *
 * Created: 14.03.2013 23:33:12
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


#ifndef TASKCONTROL_H_
#define TASKCONTROL_H_

typedef enum
{
	FS_preinit=0,	// not ready
	FS_init,	// not ready
	FS_idle,	// ready, off
	FS_autoarm, // ready to be thrown
	FS_prefly, // short pause before fly; possibly used to ramp up motors
	FS_fly,    // we fly...
	FS_init_sensor_zero, // zero the gyro // on demand only // todo move to idle; omit state
	FS_error	// error
}FlightState_t;


typedef struct IMUdata_tag
{
	int16_t roll_deg; // degrees
	int16_t pitch_deg; // degrees
	int16_t mag_heading_deg; // degrees
	int16_t height_dm; // dm = 0.1m
}IMUdata_t;

extern IMUdata_t IMUdata;

#endif /* TASKCONTROL_H_ */