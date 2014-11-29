/*
 * vector.h
 *
 * Created: 05.04.2013 19:22:05
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


#ifndef VECTOR_H_
#define VECTOR_H_

typedef struct vector3_tag
{
	float x;
	float y;
	float z;
} vector3_t;

typedef struct vector2_tag
{
	float x;
	float y;
} vector2_t;


float vector_len(vector3_t* a );
vector3_t vector_normalize( vector3_t* input );
vector3_t vector_subtract ( vector3_t* a, vector3_t* b );
vector3_t vector_scale (   vector3_t* a,  float f  );
vector3_t vector_copy( vector3_t* a );
vector3_t vector_add (  vector3_t* a, vector3_t* b );
float rectify_rad( float rad );
float vector2len(float x, float y);


#endif /* VECTOR_H_ */