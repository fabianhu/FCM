/*
 * eic.h
 * Driver for the AVR32 UC3B external interrupt controller, as ASF is way too complicated.
 *
 * Created: 27.11.2012 23:29:24
 *  
 * (c) 2013-2014 by Fabian Huslik
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


#ifndef EIC_H_
#define EIC_H_

void eic_sw_edge_rising(uint32_t pin);
void eic_sw_edge_falling(uint32_t pin);
void eic_clear_int(uint32_t pin);
void eic_enable_channel(uint32_t channel);
void eic_disable_channel(uint32_t channel);
int32_t eic_is_rising(uint32_t pin);



#endif /* EIC_H_ */