/*
 * quaternions.h
 *
 * Created: 10.02.2013 20:51:53
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


/*
http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/index.htm


A rotation is represented by:

q = cos(a/2) + i ( x * sin(a/2)) + j (y * sin(a/2)) + k ( z * sin(a/2))

where:

a=angle of rotation.
x,y,z = vector representing axis of rotation.

Rotations are combined by multiplying the quaternions representing them.


*/

#ifndef QUATERNIONS_H_
#define QUATERNIONS_H_

typedef struct quaternion_tag
{
	float w,x,y,z;
}quaternion_t;


quaternion_t quaternion_from_euler(float x, float y , float z);
void quaternion_to_euler(quaternion_t q, float* x, float* y , float* z);
void quaternion_normalize(quaternion_t* q);
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2);
quaternion_t quaternion_multiply_flip_norm(quaternion_t q1, quaternion_t q2);
quaternion_t quaternion_inverse(quaternion_t q);
void quaternion_flip(quaternion_t* q);
float invSqrt(float x);
quaternion_t quaternion_slerp(quaternion_t qa, quaternion_t qb, float t);
quaternion_t quaternion_lerp(quaternion_t qa, quaternion_t qb, float t);
quaternion_t quaternion_nlerp(quaternion_t qa, quaternion_t qb, float t);
void quaternion_copy(quaternion_t* qtrg, quaternion_t* qsrc);
vector3_t quaternion_rotateVector(vector3_t _V, quaternion_t _Q);
quaternion_t RC_Get_Offset(int32_t cmd_elev,int32_t cmd_roll,float yaw_rad);

quaternion_t perlsign16(quaternion_t qi, int n);
vector3_t perlmuttv6(vector3_t qi, int n);
vector3_t perlsignv8(vector3_t vi, int n);
quaternion_t perlmutt24(quaternion_t qi, int n);

int quaternion_test(void);
int32_t tqr(float qw,float qx,float qy,float qz,float vx,float vy,float vz);
void TheBigTest(void);

#endif /* QUATERNIONS_H_ */