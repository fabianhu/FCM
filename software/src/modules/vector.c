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


vector3_t vector_subtract (  vector3_t* a, vector3_t* b )
{
	vector3_t res;
	
	res.x = a->x - b->x;
	res.y = a->y - b->y;
	res.z = a->z - b->z;
	
	return res;
}


vector3_t vector_add (  vector3_t* a, vector3_t* b )
{
	vector3_t res;
	
	res.x = a->x + b->x;
	res.y = a->y + b->y;
	res.z = a->z + b->z;
	
	return res;
}


vector3_t vector_scale (  vector3_t* a,  float f )
{
	vector3_t res;
	
	res.x = a->x * f;
	res.y = a->y * f;
	res.z = a->z * f;
	
	return res;
}


float vector_len(vector3_t* a )
{
	return sqrtf( a->x * a->x  + a->y * a->y  + a->z * a->z  )	;
}

vector3_t vector_normalize( vector3_t* a )
{
	float magnitude ;
	vector3_t res;
	magnitude = vector_len( a ) ;
	if ( magnitude > 0 )
	{
		res.x = a->x / magnitude;
		res.y = a->y / magnitude;
		res.z = a->z / magnitude;
	}
	else
	{
		res.x=res.y=res.z=0;
	}
	return res ;
}

vector3_t vector_copy( vector3_t* a )
{
	vector3_t res;

	res.x = a->x ;
	res.y = a->y ;
	res.z = a->z ;

	return res ;
}

// adjust to +-pi
float rectify_rad( float rad )
{
	if(rad > M_PI) rad -= 2*M_PI;
	if(rad < -M_PI) rad += 2*M_PI;
	return rad;
}


float vector2len(float x, float y)
{
	return sqrt(x*x+y*y);
}

