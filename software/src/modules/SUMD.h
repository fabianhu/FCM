/*
 * SUMD.h
 * Analyze (Graupner) SUMD-frame
 * Created: 01.11.2012 11:58:58
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

#ifndef SUMD_H_
#define SUMD_H_

// Analyze (Graupner) SUMD-frame
// output: Array of uint16_t for the CR commands
// input: Array, where the HOTT-frame resides
// return the count of servos signals (0 on error in frame)

int SUMD_ConvertToServos(int16_t* output, volatile uint8_t* input);
int16_t scaleplausServoFromSUMD(int16_t raw,int16_t old);

#define SUMD_MAXCHANNELS 12
// fixme set to 16 (check the rest!!!!)

#endif /* SUMD_H_ */