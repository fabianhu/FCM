/*
 * quaternions.c
 *
 * Created: 10.02.2013 20:51:44
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
#include "../vector.h"
#include "quaternions.h"

int debug_perlmutti=0;
 
// all angles are in radians.


/*
What is the magic here ?

1. quaternion to euler and quaternion from euler should give same result.
2. the priority order is Roll-Pitch-Yaw (airplane-like!)
3. orientation of quaternion MUST be the same as in madgwicks algo
4. the directions must be right
*/
quaternion_t quaternion_from_euler(float fPitch, float fRoll, float fYaw)
{
	quaternion_t q;
	
	fRoll = -(fRoll - M_PI);
	
	fYaw = M_PI - fYaw;
	fPitch = 2*M_PI - fPitch;
	
	float fCosHYaw = cos(fYaw * .5f);
	float fSinHYaw = sin(fYaw * .5f);
	float fCosHPitch = cos(fPitch * .5f);
	float fSinHPitch = sin(fPitch * .5f);
	float fCosHRoll = cos(fRoll * .5f);
	float fSinHRoll = sin(fRoll * .5f);

// sequence matching to madgwick
	q.w = fCosHYaw * fSinHPitch * fCosHRoll + fSinHYaw * fCosHPitch * fSinHRoll;
	q.x = fSinHYaw * fCosHPitch * fCosHRoll - fCosHYaw * fSinHPitch * fSinHRoll;
	q.y = fCosHYaw * fCosHPitch * fCosHRoll + fSinHYaw * fSinHPitch * fSinHRoll;
	q.z = fCosHYaw * fCosHPitch * fSinHRoll - fSinHYaw * fSinHPitch * fCosHRoll;

	return q;
}

// Priority Roll-Pitch-Yaw
void quaternion_to_euler(quaternion_t q, float* fPitch, float* fRoll, float* fYaw)
{
	float fy  = atan2(2 * (q.y * q.x + q.w *q. z) , 1 - 2 * (q.x * q.x + q.w * q.w));
	float fp = asin(2 * (q.y * q.w - q.z * q.x) );
	float fr = atan2(2 * (q.y * q.z + q.x * q.w) , 1 - 2 * (q.w * q.w + q.z * q.z));
	
	fy = M_PI - fy;
	
	if(fy > M_PI)
	{
		fy -= 2*M_PI;
	}
	
	
	fr -= M_PI; 
	if(fr < -M_PI)
	{
		fr += 2*M_PI;
	}
	
	*fPitch = -fp;
	*fRoll= -fr;
	*fYaw = fy;
}
	

 
void quaternion_normalize(quaternion_t* q)
{
	float norm;

	norm = invSqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

	q->w = q->w * norm;
	q->x = q->x * norm;
	q->y = q->y * norm;
	q->z = q->z * norm;
}

// result is the rotation, which is needed to perform at first q1 and then q2. (the other order yields another result!)
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2) // todo rebuild to pointers
{
	quaternion_t ret;

	ret.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	ret.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	ret.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
	ret.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;

	return ret; 
}

// flip quaternion to get shortest path (otherwise we "win" a whole revolution) quaternions store 720°!
quaternion_t quaternion_multiply_flip_norm(quaternion_t q1, quaternion_t q2) // todo rebuild to pointers
{
	quaternion_t ret;

	ret = quaternion_multiply(q1,q2);
	if(ret.w < 0)
	{
		quaternion_flip(&q2);
		ret = quaternion_multiply(q1,q2);
	}	
	
	quaternion_normalize(&ret);

	return ret; 
}


quaternion_t quaternion_inverse(quaternion_t q)  // todo rebuild to pointers
{
	quaternion_t r;
	r.w = q.w;
	r.x = -q.x;
	r.y = -q.y;
	r.z = -q.z;
	return r;
}


void quaternion_flip(quaternion_t* q)
{
	q->w = -q->w;
	q->x = -q->x;
	q->y = -q->y;
	q->z = -q->z;
}




//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

// spherical linear interpolation
quaternion_t quaternion_slerp(quaternion_t qa, quaternion_t qb, float t) 
{
	// quaternion to return
	quaternion_t qm;
	// Calculate angle between them.
	float cosHalfTheta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
	// if qa=qb or qa=-qb then theta = 0 and we can return qa
	if (fabs(cosHalfTheta) >= 1.0){
		qm.w = qa.w;qm.x = qa.x;qm.y = qa.y;qm.z = qa.z;
		return qm;
	}
	// Calculate temporary values.
	float halfTheta = acosf(cosHalfTheta);
	float sinHalfTheta = sqrtf(1.0 - cosHalfTheta*cosHalfTheta);
	// if theta = 180 degrees then result is not fully defined
	// we could rotate around any axis normal to qa or qb
	if (fabs(sinHalfTheta) < 0.001)
	{ // fabs is floating point absolute
		qm.w = (qa.w * 0.5 + qb.w * 0.5);
		qm.x = (qa.x * 0.5 + qb.x * 0.5);
		qm.y = (qa.y * 0.5 + qb.y * 0.5);
		qm.z = (qa.z * 0.5 + qb.z * 0.5);
		return qm;
	}
	float ratioA = sinf((1 - t) * halfTheta) / sinHalfTheta;
	float ratioB = sinf(t * halfTheta) / sinHalfTheta;
	//calculate Quaternion.
	qm.w = (qa.w * ratioA + qb.w * ratioB);
	qm.x = (qa.x * ratioA + qb.x * ratioB);
	qm.y = (qa.y * ratioA + qb.y * ratioB);
	qm.z = (qa.z * ratioA + qb.z * ratioB);
	return qm;
}

// linear interpolation
quaternion_t quaternion_lerp(quaternion_t qa, quaternion_t qb, float t) 
{
	quaternion_t qm;
	
	qm.w = (qa.w * (1.0-t) + qb.w * t);
	qm.x = (qa.x * (1.0-t) + qb.x * t);
	qm.y = (qa.y * (1.0-t) + qb.y * t);
	qm.z = (qa.z * (1.0-t) + qb.z * t);
	
	return qm;
}	

// linear interpolation
quaternion_t quaternion_nlerp(quaternion_t qa, quaternion_t qb, float t)
{
	quaternion_t qm;
	
	quaternion_lerp(qa,qb,t);
	
	quaternion_normalize(&qm);
	
	return qm;
}


void quaternion_copy(quaternion_t* qtrg, quaternion_t* qsrc)
{
	qtrg->w = qsrc->w;
	qtrg->x = qsrc->x;
	qtrg->y = qsrc->y;
	qtrg->z = qsrc->z;
}


vector3_t quaternion_rotateVector(vector3_t _V, quaternion_t _Q)
{

//  	if(_Q.w <0)
//  	{
// 	 	quaternion_flip(&_Q);
//  	}

// 	quaternion_t _Q;
// 	_Q = perlmutt(_Qi,debug_perlmutti);
	
	// o[w]=i[z];o[x]=i[x];o[y]=i[y];o[z]=i[w];
	
	if(_Q.w <0)
	{
		quaternion_flip(&_Q);
	}
	
	float i = _Q.x;
	float j = _Q.y;
	float k = _Q.w;
	float r = _Q.z;	
	
	vector3_t vec; 
	vec.x = 2*(r*_V.z*j + i*_V.z*k - r*_V.y*k + i*_V.y*j) + _V.x*(r*r + i*i - j*j - k*k);
	vec.y = 2*(r*_V.x*k + i*_V.x*j - r*_V.z*i + j*_V.z*k) + _V.y*(r*r - i*i + j*j - k*k);
	vec.z = 2*(r*_V.y*i - r*_V.x*j + i*_V.x*k + j*_V.y*k) + _V.z*(r*r - i*i - j*j + k*k);
	return vec;
}

quaternion_t RC_Get_Offset(int32_t cmd_elev,int32_t cmd_roll,float yaw_rad)
{
	float x,y,z; // radians for this loop
	x = (float) -cmd_elev / 4000.0; // magic number by try
	y = (float) cmd_roll / 4000.0;
	z = (float) yaw_rad;
	
	return quaternion_from_euler(x,y,z);
}


quaternion_t perlmutt(quaternion_t qi, int n)
{
	float i[4];
	float o[4];
	
	i[0] = qi.w;
	i[1] = qi.x;
	i[2] = qi.y;
	i[3] = qi.z;
	
	switch(n)
	{
		case 1: o[0]=i[0];o[1]=i[1];o[2]=i[2];o[3]=i[3]; break;
		case 2: o[0]=i[0];o[1]=i[1];o[2]=i[3];o[3]=i[2]; break;
		case 3: o[0]=i[0];o[1]=i[2];o[2]=i[1];o[3]=i[3]; break;
		case 4: o[0]=i[0];o[1]=i[2];o[2]=i[3];o[3]=i[1]; break;
		case 5: o[0]=i[0];o[1]=i[3];o[2]=i[1];o[3]=i[2]; break;
		case 6: o[0]=i[0];o[1]=i[3];o[2]=i[2];o[3]=i[1]; break;
		case 7: o[0]=i[1];o[1]=i[0];o[2]=i[2];o[3]=i[3]; break;
		case 8: o[0]=i[1];o[1]=i[0];o[2]=i[3];o[3]=i[2]; break;
		case 9: o[0]=i[1];o[1]=i[2];o[2]=i[0];o[3]=i[3]; break;
		case 10: o[0]=i[1];o[1]=i[2];o[2]=i[3];o[3]=i[0]; break;
		case 11: o[0]=i[1];o[1]=i[3];o[2]=i[0];o[3]=i[2]; break;
		case 12: o[0]=i[1];o[1]=i[3];o[2]=i[2];o[3]=i[0]; break;
		case 13: o[0]=i[2];o[1]=i[0];o[2]=i[1];o[3]=i[3]; break;
		case 14: o[0]=i[2];o[1]=i[0];o[2]=i[3];o[3]=i[1]; break;
		case 15: o[0]=i[2];o[1]=i[1];o[2]=i[0];o[3]=i[3]; break;
		case 16: o[0]=i[2];o[1]=i[1];o[2]=i[3];o[3]=i[0]; break;
		case 17: o[0]=i[2];o[1]=i[3];o[2]=i[0];o[3]=i[1]; break;
		case 18: o[0]=i[2];o[1]=i[3];o[2]=i[1];o[3]=i[0]; break;
		case 19: o[0]=i[3];o[1]=i[0];o[2]=i[1];o[3]=i[2]; break;
		case 20: o[0]=i[3];o[1]=i[0];o[2]=i[2];o[3]=i[1]; break;
		case 21: o[0]=i[3];o[1]=i[1];o[2]=i[0];o[3]=i[2]; break;
		case 22: o[0]=i[3];o[1]=i[1];o[2]=i[2];o[3]=i[0]; break;
		case 23: o[0]=i[3];o[1]=i[2];o[2]=i[0];o[3]=i[1]; break;
		case 24: o[0]=i[3];o[1]=i[2];o[2]=i[1];o[3]=i[0]; break;
		default: o[0]=1;o[1]=0;o[2]=0;o[3]=0; break;

	}
	
	quaternion_t ret;
	ret.w = o[0];
	ret.x = o[1];
	ret.y = o[2];
	ret.z = o[3];
	
	return ret;


}


//////////////////////////////

/*

//
// Quaternion Class
//
#ifndef CQuat_h
#define CQuat_h

const float TO_HALF_RAD = 3.14159265f / 360.0f;
const float MINVAL = 0.005f;

class CQuat
{
public:
	float x,y,z,w;

	CQuat( ) : x(0), y(0), z(0), w(1)
	{
	}

	CQuat( float fx, float fy, float fz, float fw ) : x(fx), y(fy), z(fz), w(fw)
	{
	}

	// This just took four floats initially to avoid dependence on the vector class
	// but I decided avoiding confusion with the value setting constructor was more important
	CQuat( float Angle, const CVec3& Axis )
	{
		SetAxis( Angle, Axis.x, Axis.y, Axis.z );
	}

	// No rotation
	void Reset( )
	{
		x = 0;
		y = 0;
		z = 0;
		w = 1;
	}

	// Set Quat from axis-angle
	void SetAxis( float degrees, float fX, float fY, float fZ )
	{
		float HalfAngle = degrees * TO_HALF_RAD; // Get half angle in radians from angle in degrees
		float sinA = (float)sin( HalfAngle ) ;
		w = (float)cos( HalfAngle );
		x = fX * sinA;
		y = fY * sinA;
		z = fZ * sinA;
	}

	CQuat Invert( ) const
	{
		return CQuat( -x, -y, -z, w );
	}

	// Note that order matters with concatenating Quaternion rotations
	inline CQuat operator* (const CQuat &b) const
	{
		CQuat r;

		r.w = w*b.w - x*b.x  -  y*b.y  -  z*b.z;
		r.x = w*b.x + x*b.w  +  y*b.z  -  z*b.y;
		r.y = w*b.y + y*b.w  +  z*b.x  -  x*b.z;
		r.z = w*b.z + z*b.w  +  x*b.y  -  y*b.x;

		return r;
	}
	
	// You could add an epsilon to this equality test if needed
	inline bool operator== ( const CQuat &b ) const
	{
		return (x == b.x && y == b.y && z == b.z && w == b.w);
	}

	int IsIdentity( ) const
	{
		return (x == 0.0f && y == 0.0f && z == 0.0f && w==1.0f);
	}

	// Can be used the determine Quaternion neighbourhood
	float Dot( const CQuat& a ) const
	{
		return x * a.x + y * a.y + z * a.z + w * a.w;
	}

	// Scalar multiplication
	CQuat operator*( float s ) const
	{
		return CQuat(x * s, y * s, z * s, w * s );
	}

	// Addition
	CQuat operator+ ( const CQuat& b ) const
	{
		return CQuat( x + b.x, y + b.y, z + b.z, w + b.w );
	}

	// ------------------------------------
	// Simple Euler Angle to Quaternion conversion, this could be made faster
	// ------------------------------------
	void FromEuler( float rx, float ry, float rz )
	{
		CQuat qx(-rx, CVec3( 1, 0, 0 ) );
		CQuat qy(-ry, CVec3( 0, 1, 0 ) );
		CQuat qz(-rz, CVec3( 0, 0, 1 ) );
		qz = qy * qz;
		*this = qx * qz;
	}

	// ------------------------------------
	// Quaternions store scale as well as rotation, but usually we just want rotation, so we can normalize.
	// ------------------------------------
	int Normalize( )
	{
		float lengthSq = x * x + y * y + z * z + w * w;

		if (lengthSq == 0.0 ) return -1;
		if (lengthSq != 1.0 )
			{
			float scale = ( 1.0f / sqrtf( lengthSq ) );
			x *= scale;
			y *= scale;
			z *= scale;
			w *= scale;
			return 1;
			}
		return 0;
	}

	// ------------------------------------
	// Creates a value for this Quaternion from spherical linear interpolation
	// t is the interpolation value from 0 to 1
	// if bReduceTo360 is true, the interpolation will take the shortest path for a 360 deg angle range (max delta rotation = 180 degrees)
	// if bReduceTo360 is false, the interpolation will take the shortest path for a 720 deg angle range (max delta rotation = 360 degrees)
	// ------------------------------------
	void Slerp(const CQuat& a, const CQuat& b, float t, const bool bReduceTo360 )
	{
	  float w1, w2;
	  int bFlip = 0;
	
	  float cosTheta = a.Dot(b);
	  if ( bReduceTo360 && cosTheta < 0.0f ) { // We need to flip a quaternion for shortest path interpolation
		  cosTheta = -cosTheta; 
		  bFlip = 1;
	  }
	  float theta    = acos(cosTheta);
	  float sinTheta = sin(theta);
	
	  if( sinTheta > MINVAL )
	  {
	    w1 = sin( (1.0f-t)*theta ) / sinTheta;
	    w2 = sin( t*theta) / sinTheta;
	  } else {
	    // They're almost the same quaternion
	    w1 = 1.0f - t;
	    w2 = t;
	  }
	
	  if ( bFlip ) 
		  w2 = -w2;
	
	  *this = a*w1 + b*w2;
	}

	// ------------------------------------
	// linearly interpolate each component, then normalize the Quaternion
	// Unlike spherical interpolation, this does not rotate at a constant velocity,
	// although that's not necessarily a bad thing
	// ------------------------------------
	void Nlerp(const CQuat &a, const CQuat &b, float t, const bool bReduceTo360 )
	{
		float t1 = 1.0f - t;

		if ( bReduceTo360 && a.Dot(b) < 0.0f )
			*this = a * t1 + b * -t;
		else
			*this = a * t1 + b * t;

		Normalize();
	}

	// ------------------------------------
	// Set a 4x4 matrix with the rotation of this Quaternion
	// ------------------------------------
	void inline ToMatrix( float mf[16] ) const
	{
		float x2 = 2.0f * x,  y2 = 2.0f * y,  z2 = 2.0f * z;

		float xy = x2 * y,  xz = x2 * z;
		float yy = y2 * y,  yw = y2 * w;
		float zw = z2 * w,  zz = z2 * z;

		mf[ 0] = 1.0f - ( yy + zz );
		mf[ 1] = ( xy - zw );
		mf[ 2] = ( xz + yw );
		mf[ 3] = 0.0f;

		float xx = x2 * x,  xw = x2 * w,  yz = y2 * z;

		mf[ 4] = ( xy +  zw );
		mf[ 5] = 1.0f - ( xx + zz );
		mf[ 6] = ( yz - xw );
		mf[ 7] = 0.0f;

		mf[ 8] = ( xz - yw );
		mf[ 9] = ( yz + xw );
		mf[10] = 1.0f - ( xx + yy );  
		mf[11] = 0.0f;  

		mf[12] = 0.0f;  
		mf[13] = 0.0f;   
		mf[14] = 0.0f;   
		mf[15] = 1.0f;
	}

	// ------------------------------------
	// Set this Quat to aim the Z-Axis along the vector from P1 to P2
	// ------------------------------------
	void AimZAxis( const CVec3& P1, const CVec3& P2 )
	{
		CVec3 vAim = P2 - P1;
		vAim.Normalize();

		x = vAim.y;
		y = -vAim.x;
		z = 0.0f;
		w = 1.0f + vAim.z;

		if ( x == 0.0f && y == 0.0f && z == 0.0f && w == 0.0f ) {
			*this = CQuat( 0, 1, 0, 0 ); // If we can't normalize it, just set it
		} else {
			Normalize();
		}
	}
};

#endif
*/

/*
const float pi = 3.14159;

void quaternion2euler(float* q0,float* q1,float* q2, float* q3,float* phi,float* theta,float* psi)
{ 
	*phi = atan2(2*(*q0 * *q1+*q2 * *q3),1-2*(pow(*q1,2)+pow(*q2,2)))/pi*180;                          //So sieht der Prozeduraufruf aus
	*theta=asin(2*(*q0 * *q2-*q3 * *q1))/pi*180;                                                       //Einfach die Formeln nachgebildet
	*psi = atan2(2*(*q0 * *q3+*q1 * *q2),1-2*(pow(*q2,2)+pow(*q3,2)))/pi*180;                          //Es werden Grad zurückgegeben
}


void QuaternionToEuler(const dQuaternion quaternion, vector3df &euler)
{
    dReal w,x,y,z;

    w = quaternion[0];
    x = quaternion[1];
    y = quaternion[2];
    z = quaternion[3];

    double sqw = w*w;    
    double sqx = x*x;    
    double sqy = y*y;    
    double sqz = z*z; 

    euler.Z = (irr::f32) (atan2(2.0 * (x*y + z*w),(sqx - sqy - sqz + sqw)) * (180.0f/irr::core::PI));
    euler.X = (irr::f32) (atan2(2.0 * (y*z + x*w),(-sqx - sqy + sqz + sqw)) * (180.0f/irr::core::PI));          
    euler.Y = (irr::f32) (asin(-2.0 * (x*z - y*w)) * (180.0f/irr::core::PI));

}












====================
CoreCode1-From EulerAngle To Quaternion:

void Quaternion::FromEulerAngle(const EulerAngle &ea)
{
        float fCosHRoll = cos(ea.fRoll * .5f);
        float fSinHRoll = sin(ea.fRoll * .5f);
        float fCosHPitch = cos(ea.fPitch * .5f);
        float fSinHPitch = sin(ea.fPitch * .5f);
        float fCosHYaw = cos(ea.fYaw * .5f);
        float fSinHYaw = sin(ea.fYaw * .5f);

        w = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
        x = fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
        y = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
        z = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
}


CoreCode2-From  Quaternion To EulerAngle


EulerAngle Quaternion::ToEulerAngle() const
{
        EulerAngle ea;

        ea.fRoll  = atan2(2 * (w * z + x * y) , 1 - 2 * (z * z + x * x));
        ea.fPitch = asin(limit(2 * (w * x - y * z) , -1.0f , 1.0f));
        ea.fYaw   = atan2(2 * (w * y + z * x) , 1 - 2 * (x * x + y * y));

        return ea;
}


Corecode3-Quaternion multiplication

Quaternion Quaternion::Multiply(const Quaternion &b)
{
        Quaternion c;
        c.w=w*b.w        -x*b.x        -y*b.y        -z*b.z;
        c.x=w*b.x        +x*b.w        +y*b.z        -z*b.y;
        c.y=w*b.y        -x*b.z        +y*b.w        +z*b.x;
        c.z=w*b.z        +x*b.y        -y*b.x        +z*b.w;
        c.Normalize();
        return c;

Common code1:Normalization of Quaternion

void  Quaternion::Normalize(){
        float s=getS();
        w/=s;
        x/=s;
        y/=s;
        z/=s;
}
float Quaternion::getS(){
        return sqrt(w*w+x*x+y*y+z*z);
}


	




function a = quaternionToEuler(q)
% a = quaternionToEuler(q)

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

a(1) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
a(2) = arcsin(2*(q0*q2 - q3*q1));
a(3) = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));


function q = eulerToQuaternion(a)
% q = eulerToQuaternion(a)

if size(a,1) == 3
    a = a';
end

c1 = cos(a(:,1)/2);
c2 = cos(a(:,2)/2);
c3 = cos(a(:,3)/2);
s1 = sin(a(:,1)/2);
s2 = sin(a(:,2)/2);
s3 = sin(a(:,3)/2);

q = [c1.*c2.*c3 + s1.*s2.*s3, s1.*c2.*c3 - c1.*s2.*s3, c1.*s2.*c3 + s1.*c2.*s3, c1.*c2.*s3 - s1.*s2.*c3];



*/	