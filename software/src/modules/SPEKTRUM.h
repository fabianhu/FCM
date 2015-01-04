/*
 * SPEKTRUM.h
 *
 * Created: 18.11.2012 01:07:24
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


#ifndef SPEKTRUM_H_
#define SPEKTRUM_H_

// bind mode pulses
// 2 low pulses: DSM2 1024/22ms
// 4 low pulses: DSM2 2048/11ms
// 6 low pulses: DSMX 22ms
// 8 low pulses: DSMX 11ms

#if SPEKTRUM_DSMX == 1 // fixme old stuff
	#define SPEKTRUM_BITS 2048
	#define SPEKTRUM_BIND_PULSES 8
#else 
	#define SPEKTRUM_BITS 1024 // only a simplification
	#define SPEKTRUM_BIND_PULSES 2
#endif

#define SPEKTRUM_MAX_CHANNEL 8
#define SPEKTRUM_FRAME_SIZE 16
#if (SPEKTRUM_BITS == 1024)
#define SPEKTRUM_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
#define SPEKTRUM_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
#endif
#if (SPEKTRUM_BITS == 2048)
#define SPEKTRUM_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
#define SPEKTRUM_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
#endif


int SPEKTRUM_ConvertToServos(int16_t* output, volatile uint8_t* input);
int16_t scaleplausServoFromSPEKTRUM(uint16_t raw, int16_t old);
void SPEKTRUM_Bind(void);

#endif /* SPEKTRUM_H_ */