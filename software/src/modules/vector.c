/*
 * vector.c
 *
 * Created: 05.04.2013 19:21:48
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
#include <fastmath.h>
#include "types.h"
#include "vector.h"


vector3_t vector_subtract (  vector3_t* srcV1, vector3_t* srcV2 )
{
	vector3_t dstV;
	
	dstV.x = srcV1->x - srcV2->x;
	dstV.y = srcV1->y - srcV2->y;
	dstV.z = srcV1->z - srcV2->z;
	
	return dstV;
}


vector3_t vector_add (  vector3_t* srcV1, vector3_t* srcV2 )
{
	vector3_t dstV;
	
	dstV.x = srcV1->x + srcV2->x;
	dstV.y = srcV1->y + srcV2->y;
	dstV.z = srcV1->z + srcV2->z;
	
	return dstV;
}


vector3_t vector_scale (  vector3_t* srcV,                    /* ptr to source vector */
float sclVal                    /* scale value (Q.15 fractional) */
)
{
	vector3_t dstV;
	
	dstV.x = srcV->x * sclVal;
	dstV.y = srcV->y * sclVal;
	dstV.z = srcV->z * sclVal;
	
	return dstV;
}


float vector_len(vector3_t* in )
{
	return sqrtf( in->x * in->x  + in->y * in->y  + in->z * in->z  )	;
}

vector3_t vector_normalize( vector3_t* input )
{
	float magnitude ;
	vector3_t result;
	magnitude = vector_len( input ) ;
	if ( magnitude > 0 )
	{
		result.x = input->x / magnitude;
		result.y = input->y / magnitude;
		result.z = input->z / magnitude;
	}
	else
	{
		result.x=result.y=result.z=0;
	}
	return result ;
}

vector3_t vector_copy( vector3_t* input )
{
	vector3_t result;

	result.x = input->x ;
	result.y = input->y ;
	result.z = input->z ;

	return result ;
}

// adjust to +-pi
float rectify_rad( float fSetHeading )
{
	if(fSetHeading > M_PI) fSetHeading -= 2*M_PI;
	if(fSetHeading < -M_PI) fSetHeading += 2*M_PI;
	return fSetHeading;
}



float vector2len(float x, float y)
{
	return sqrt(x*x+y*y);
}

/*
float vector2angle(float x, float y) ccw rotation
{
	volatile float angle;
	
	angle = atan2(y, x);
	
	if (angle < 0.0) 
	angle = angle + 2 * M_PI;
	
	if (angle >=  2 * M_PI) 
	angle = angle - 2 * M_PI;
	
	return angle;
}
*/


/*vector_t vector_copy_result(results_t* input )
{
	vector_t result;

	result.x = input->x ;
	result.y = input->y ;
	result.z = input->z ;

	return result ;
}*/

/*
fractional* VectorMultiply (     // Vector elem-to-elem multiply 
							// dstV[elem] =                 
							//    = srcV1[elem] * srcV2[elem] 
							// (in place capable) 
							// (with itself capable) 
							int16_t numElems,                        // number elements in srcV[1,2] (N) 
							fractional* dstV,                    // ptr to destination vector 
							fractional* srcV1,                   // ptr to source vector one 
							fractional* srcV2                    // ptr to source vector two 
							
							// dstV returned 
							)
{
	int16_t i;
	for (i=0; i< numElems; i++) {
		dstV[i] = fl2fr(fr2fl(srcV1[i]) * fr2fl(srcV2[i]));
	}
	return dstV;
}


fractional VectorDotProduct (    // Vector dot product 
							 // dotVal =                     
							 //    = sum(srcV1[elem]*srcV2[elem]) 
							 // (with itself capable) 
							 int16_t numElems,                        // number elements in srcV[1,2] (N) 
							 fractional* srcV1,                   // ptr to source vector one 
							 fractional* srcV2                    // ptr to source vector two 
							 
							 // dot product value returned 
							 )
{
	fractional sum = 0;
	int16_t i;
	for (i=0; i< numElems; i++) {
		sum += fl2fr(fr2fl(srcV1[i]) * fr2fl(srcV2[i]));
	}
	return sum;
}


fractional VectorPower (         // Vector power 
						// powVal =                     
						//    = sum(srcV[elem]^2)       
						int16_t numElems,                       // number elements in srcV (N) 
						fractional* srcV                     // ptr to source vector one
						
						// power value returned 
						)
{
	fractional sum = 0;
	int16_t i;
	for (i=0; i< numElems; i++) {
		sum += fl2fr(fr2fl(srcV[i]) * fr2fl(srcV[i]));
	}
	return sum;
}
*/