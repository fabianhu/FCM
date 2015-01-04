/*
 * servo_in.h
 *
 * Created: 08.01.2012 21:31:41
 *
 * (c) 2012-2015 by Fabian Huslik
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


#ifndef SERVO_IN_H_
#define SERVO_IN_H_

typedef enum RXType_tag
{
	RXType_Unknown = 0,
	RXType_Classic, // 4 PPM signals
	RXType_PPM7, // PPM sum signal
	RXType_SerialUnknown, 
	RXType_SUMD, // HoTT Digital format
	RXType_SPEKTRUM // SPEKTRUM digital format
}RXType_t;

extern int16_t servo_in_channel_raw[];
extern RXType_t RXType;



void servo_in_init(uint8_t AlarmID);
extern int servo_in_getCmd(int32_t* thr, int32_t* roll, int32_t* elev, int32_t* yaw);
int32_t servo_in_get_ext_channel(int ch, int32_t* value);
bool servo_in_get_ext_channel_switch(int ch, int SwP);
bool servo_in_Start(int32_t cmd_thr, int32_t cmd_roll, int32_t cmd_elev, int32_t cmd_yaw);

#endif /* SERVO_IN_H_ */