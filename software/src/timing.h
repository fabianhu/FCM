/*
 * timing.h
 *
 * Created: 10.01.2012 21:52:48
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


#ifndef TIMING_H_
#define TIMING_H_

#define TIMERCHANNEL_RC 0
#define TIMERCHANNEL_OS 1
#define TIMERCHANNEL_RC2 2

#define RC_IN_TIMER_COUNT_1MS 1500L // depends on timer

extern void RC_in_timing_init(void);





#endif /* TIMING_H_ */