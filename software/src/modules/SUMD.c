/*
 * SUMD.c
 *
 * Created: 01.11.2012 11:58:50
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

#include <asf.h>
#include "SUMD.h"
#include "CRC_CCITT.h"

// Analyze (Graupner) SUMD-frame
// output: Array of uint16_t for the CR commands
// input: Array, where the HOTT-frame resides
// return the count of servos signals (0 on error in frame)
// Servo signal is normalized to -5000..5000

// input: serial frame
// output: servo signals
int SUMD_ConvertToServos(int16_t* output, volatile uint8_t* input)
{
	int ret = 0; // return 0 on error
	uint16_t usCRC;
	uint8_t ChanCount = input[2];
		
	// 0 = always 168
	// 1 = seems to be a kind of error indicator (1 normally, 129 if TX was off at start)
	// 2 = count of following channels
	// 3 = high byte of Servo 0
	// 4 = low byte of Servo 0; raw min is 8000 mid is 12000, max is 16000 at 120% at TX.
	// ... repeats over the channel count
	// n-1 = High byte of CRC
	// n = Low byte of CRC16, which is CRC-CCITT (XModem)  with 0x1021 as polynomial and 0x0000 as starting value.
		
		
	if (input[0]==168 && ChanCount >=6 && ChanCount <= 12) // seems to be a valid HOTT-SUMD frame
	{
		// check CRC-CCITT (XModem) checksum:
		// CRC-CCITT (XModem)  with 0x1021 as polynom and 0x0000 as starting value.
		// Reverse engineered by FH using LibreOffice-Calc and http://www.lammertbies.nl/comm/info/crc-calculation.html.
			
		usCRC = calc_crc_ccitt(input,3+ChanCount*2);
		
		uint16_t crccomp = input[3+ChanCount*2]<<8 | input[4+ChanCount*2];
		
		if(usCRC == crccomp )
		{
			for (int i = 0;i<ChanCount;i++)
			{
				output[i] = input[i*2+4] | input[i*2+3]<<8;
			}
			for (int i = ChanCount;i<SUMD_MAXCHANNELS;i++)
			{
				output[i] = 0; // zero for the rest
			}
			ret = ChanCount;
		}
	}
	
	return ret;
	
	
}

// normalize to 0..10000
int16_t scaleplausServoFromSUMD(int16_t raw, int16_t old)
{
	// raw min is 8800 mid is 12000, max is 15200 for 100%
	int32_t res = 0;
	
	if (raw < 8000 || raw > 16000)
	{
		// failed measurement; return zero.
		res = old;
	}
	else
	{
		res = (((int32_t)raw -12000)*5)/4; // gives -5000..5000 for 120% of output.
	}
	
	return (int16_t)res;
}