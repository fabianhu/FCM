/*
 * magnetics_calibration.c
 *
 * Created: 16.03.2013 22:48:07
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

#include <asf.h>
#include "types.h"
#include "vector.h"
#include "magnetics_calibration.h"
#include <fastmath.h>

/*
How this works:
We assume, that the measured field is uniform.
We assume, that the sensitivity of the sensor is about the same for all axes.
This leads to the assumption, that all points of measurement are placed on a sphere (kind of..)

The rest of the algorithm is described in the single steps below.
This is an iterative process. the center of the sphere gets approximated over time.

*/

#define MAGCOMPFACTOR 0.25 // todo make dynamical, taking number of measurements into account!
#define MAGCOMPMINCHANGE 40000  // minimum change in vector difference from last measurement, at which a new measurement is taken
#define MAGCOMPMINCYCLES 15 // minimum cycles, at which the change must be large enough

int32_t magdebug = 0;

// return calibrated value
vector3_t magcal_compensate(vector3_t* pMagRaw, vector3_t* magOffset,bool DoCal)
{
	float magRawLen;
	vector3_t vectorChange;
	vector3_t vectorResult;
	vector3_t vectorCompensated;
	vector3_t vectorOffsetEstimate;
	static uint32_t ctr;	// counter to decide, if it is worth it to execute the filter
	
	static float magRawLenPrevious; // vector length from previous run
	static vector3_t magRawPrevious; // vector from previous run
	
	vectorCompensated =  vector_add(pMagRaw,magOffset); // all further calculations should be based on the compensated vector.
	
	vectorChange = vector_subtract( &vectorCompensated , &magRawPrevious ) ; // vector change since last call // anyway needed for calculation
	
	magdebug =vector_len(&vectorChange);
	if(magdebug > MAGCOMPMINCHANGE) // check, if the difference between old and new is sufficient. // fixme avoid calibrating with first values
	{
		ctr++;
	}
	else
	{
		ctr = 0;
	}
	
	
	if(ctr > MAGCOMPMINCYCLES && DoCal) // check, if the difference between old and new is sufficient and was for the last 10 cycles.
	{
		ctr = 0; // reset to prevent multiple calculations with similar values
	
		magRawLen = vector_len( &vectorCompensated ) ; // length of raw vector
		vectorChange = vector_normalize( &vectorChange ) ; // normalize to direction only
		vectorOffsetEstimate = vector_scale(&vectorChange , magRawLen - magRawLenPrevious ) ; // apply the "length error" to this direction
	
		// remember old values
		magRawLenPrevious = magRawLen;
		magRawPrevious = vector_copy(&vectorCompensated); // den hier hatten wir beim ersten mal vergessen...
	
		// apply a little quantity of the offset to the filtered offset. 
		// so all small length errors from different directions accumulate to one "good" value.
		vectorOffsetEstimate = vector_scale(&vectorOffsetEstimate,MAGCOMPFACTOR);
		*magOffset = vector_subtract(magOffset,&vectorOffsetEstimate);

	}
	
	vectorResult = vector_add(pMagRaw,magOffset);	// calculate again with the possibly better offset.
	return vectorResult;
}



