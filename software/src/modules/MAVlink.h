/*
 * MAVlink.h
 *
 * Created: 07.02.2014 20:32:41
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


#ifndef MAVLINK_H_
#define MAVLINK_H_

#define MAVMAXABOS 5 // max number of concurrent stream abos

#define MAVSYSTEMID 20 // the system ID
#define MAVCOMPONENT MAV_COMP_ID_IMU

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define NATIVE_BIG_ENDIAN

#define MAVLINK_SEND_UART_BYTES(chan,buf,len) MAV_SendAndWait(buf,len) // definition for the _send commands. // fixme put chan through ?


typedef struct MAVAbo_tag
{
	uint16_t rate; // the desired rate in ms/frame
	uint8_t stream_ID; // the MAVlink ID
	uint8_t active; // is active
	uint32_t timestamp; // last send time
} MAVAbo_t;

typedef struct MAVparam_tag
{
	int16_t* pValue;
	int16_t upper;
	int16_t lower;
	char name[17];
}MAVparam_t;


void MAV_Process(void); // requires cyclic call (with no excessive wait around)
void MAV_CopyPacket(uint8_t* ptr, uint32_t len);
void MAV_init(void);
void MAV_SendAndWait(const uint8_t* buf, uint32_t len);

#endif /* MAVLINK_H_ */