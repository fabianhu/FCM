/*
from:
https://github.com/madcowswe/Mavrx-firmware/blob/master/Thalamus/filter.h
License: unknown
works: unknown

*/


#include <asf.h>
#include <fastmath.h>
#include "../vector.h"
#include "quaternions.h"
#include "MadgwickAHRS.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "../menu/menu.h"
#include "../menu/menu_variant.h"
#include "AHRS2.h"

#define DRIFT_AccelKp 0.1
#define DRIFT_MagKp   0.1
#define FAST_RATE 0.01

float M1,M2,M3,M4,M5,M6,M7,M8,M9;
vector3_t Gyroerror = {0,0,0};

quaternion_t AHRS2(vector3_t Gyro, vector3_t Accel, vector3_t Mag)
{

// ****************************************************************************
// *** ATTITUDE HEADING REFERENCE SYSTEM
// ****************************************************************************

// CREATE THE MEASURED ROTATION MATRIX //

//Both mag and acc are normalized in their Read functions

// Compose the Measured Rotation Matrix from Accelerometer and Magnetometer Vectors
// Global Z axis in Local Frame/ RM Third Row - Accelerometer (already normalised)

float RM7,RM8,RM9,RM6,RM5,RM4,RM1,RM2,RM3;

float q1,q2,q3,q4;

RM7 = Accel.x;
RM8 = Accel.y;
RM9 = Accel.z;

// Global Y axis in local frame/ RM Second Row - Acclerometer cross Magnetometer
RM4 = Accel.y*(Mag.z) - (Accel.z)*Mag.y;
RM5 = (Accel.z)*Mag.x - Accel.x*(Mag.z);
RM6 = Accel.x*Mag.y - Accel.y*Mag.x;
// Normalise
float temp = sqrt(RM4*RM4 + RM5*RM5 + RM6*RM6);
RM4 = RM4/temp;
RM5 = RM5/temp;
RM6 = RM6/temp;

// Global X axis in Local Frame/ RM First Row - Y axis cross Z axis
RM1 = RM5*RM9 - RM6*RM8;
RM2 = RM6*RM7 - RM4*RM9;
RM3 = RM4*RM8 - RM5*RM7;
// Normalise
temp = sqrt(RM1*RM1 + RM2*RM2 + RM3*RM3);
RM1 = RM1/temp;
RM2= RM2/temp;
RM3 = RM3/temp;


// CREATE THE ESTIMATED ROTATION MATRIX //
// The measured quaternion updates the estimated quaternion via the Gyro.*.error terms applied to the gyros
float g1 = (Gyro.x - Gyroerror.x*DRIFT_AccelKp)/(float)FAST_RATE;
float g2 = (Gyro.y - Gyroerror.y*DRIFT_AccelKp)/(float)FAST_RATE;
float g3 = (Gyro.z - Gyroerror.z*DRIFT_MagKp)/(float)FAST_RATE;

// Increment the Estimated Rotation Matrix by the Gyro Rate
//First row is M1, M2, M3 - World X axis in local frame
M1 = M1 + g3*M2 - g2*M3;
M2 = M2 - g3*M1 + g1*M3;
M3 = M3 + g2*M1 - g1*M2;

//Second row is M4, M5, M6 - World Y axis in local frame
M4 = M4 + g3*M5 - g2*M6;
M5 = M5 - g3*M4 + g1*M6;
M6 = M6 + g2*M4 - g1*M5;

// We use the dot product and both X and Y axes to adjust any lack of orthogonality between the two.
float MX_dot_MY = M1*M4 + M2*M5 + M3*M6;
M1 = M1 - M4*(MX_dot_MY/2);
M2 = M2 - M5*(MX_dot_MY/2);
M3 = M3 - M6*(MX_dot_MY/2);

float sumsqu = invSqrt(M1*M1 + M2*M2 + M3*M3);
M1 = M1*sumsqu;
M2 = M2*sumsqu;
M3 = M3*sumsqu;

M4 = M4 - M1*(MX_dot_MY/2);
M5 = M5 - M2*(MX_dot_MY/2);
M6 = M6 - M3*(MX_dot_MY/2);

sumsqu = invSqrt(M4*M4 + M5*M5 + M6*M6);
M4 = M4*sumsqu;
M5 = M5*sumsqu;
M6 = M6*sumsqu;

//We find the Z axis by calculating the cross product of X and Y
M7 = M2*M6 - M3*M5;
M8 = M3*M4 - M1*M6;
M9 = M1*M5 - M2*M4;

sumsqu = invSqrt(M7*M7 + M8*M8 + M9*M9);
M7 = M7*sumsqu;
M8 = M8*sumsqu;
M9 = M9*sumsqu;

// CALCULATE GYRO BIAS //
// The gyro errores adjust the estimated matrix towards the measured rotation matrix.
// Use x and y components of the Cross Product between the z vector from each Rotation Matrix for Gyro Biases
Gyroerror.x = RM9*M8 - RM8*M9;
Gyroerror.y = RM7*M9 - RM9*M7;

// Use z component of the cross product between the x vector from each rotation matrix to create the Z gyro error
Gyroerror.z = RM2*M1 - RM1*M2;


// CALCULATE THE ESTIMATED QUATERNION //
float trace = M1 + M5 + M9;
if( trace > 0 ) {
float s = 0.5f / sqrt(trace + 1.0f);
q1 = 0.25f / s;
q2 = ( M6 - M8 ) * s;
q3 = ( M7 - M3 ) * s;
q4 = ( M2 - M4 ) * s;
}
else {
if ( M1 > M5 && M1 > M9 ) {
float s = 2.0f * sqrt( 1.0f + M1 - M5 - M9);
q1 = (M6 - M8 ) / s;
q2 = 0.25f * s;
q3 = (M4 + M2 ) / s;
q4 = (M7 + M3 ) / s;
}
else if (M5 > M9) {
float s = 2.0f * sqrt( 1.0f + M5 - M1 - M9);
q1 = (M7 - M3 ) / s;
q2 = (M4 + M2 ) / s;
q3 = 0.25f * s;
q4 = (M8 + M6 ) / s;
}
else {
float s = 2.0f * sqrt( 1.0f + M9 - M1 - M5 );
q1 = (M2 - M4 ) / s;
q2 = (M7 + M3 ) / s;
q3 = (M8 + M6 ) / s;
q4 = 0.25f * s;
}
}	

q1 = q1;
q2 = -q2;
q3 = -q3;
q4 = -q4;

// renormalise using fast inverse sq12re root
sumsqu = invSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
q1 *= sumsqu;
q2 *= sumsqu;
q3 *= sumsqu;
q4 *= sumsqu;


quaternion_t q;
q.w = q1;
q.x = q2;
q.y = q3;
q.z = q4;

q = perlmutt(q,myPar.test_out.sValue);

return q;

}