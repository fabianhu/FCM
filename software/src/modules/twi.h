/*
 * MyTwi.h
 *
 * Created: 11.10.2012 20:10:10
 *
 * (c) 2012-2014 by Fabian Huslik
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

#ifndef MYTWI_H_
#define MYTWI_H_

// fixme debug queue with last actions and times
typedef struct twitrnsf_tag
{
	uint8_t device;
	uint8_t addr;
	uint8_t len;
	uint32_t lastTime;
}twitrnsf_t;

void TWI_init(void);

void TWI_read_ISR(uint8_t device, uint8_t addr, uint8_t len, uint8_t* dest); // non blocking read a block // configure read queue and fire transfer.
void TWI_write_ISR(uint8_t device, uint8_t addr, uint8_t len, uint8_t* src); // non blocking write a block // configure write queue and fire transfer.  

void TWI_RegisterReadyHandler(vfpv_t fp);


#endif /* MYTWI_H_ */