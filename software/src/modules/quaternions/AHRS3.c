/*
 * AHRS3.c
 *
// ****************************************************************************
// *** Copyright (c) 2011, Universal Air Ltd. All rights reserved.
// *** Source and binaries are released under the BSD 3-Clause license
// *** See readme_forebrain.txt files for the text of the license
// ****************************************************************************
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
#include "AHRS3.h"

typedef struct{
	float value;
	signed short raw;
	signed short history[GYRO_AVERAGING];
	signed int total;
	float av;
	float offset;
	float si;
	float error;
	float verterror;
	float verterrorInt;
} sensorStruct;

typedef struct{
	sensorStruct X;
	sensorStruct Y;
	sensorStruct Z;
	unsigned short count;
} threeAxisSensorStruct;

threeAxisSensorStruct Gyro;
threeAxisSensorStruct Accel;
threeAxisSensorStruct Magneto;

typedef struct{
	float u;
	float a;
	float i;
	float r;
} quaternionStruct;

typedef struct {
	unsigned int raw;
	unsigned int history[BARO_AVERAGING];
	unsigned int total;
	unsigned short count;
	unsigned short temperature;
	float pressure;
	float altitude;
	float altitudeLast;
	float altitudeOffset;
	unsigned char reading;
	unsigned char prescale;
} BaroStruct;

BaroStruct Baro;
quaternionStruct quaternion,quaternionOld;
float psiAngle, thetaAngle, phiAngle;
float ahrsBarometer;
int ahrsSoftscale = 0;


quaternion_t AHRS3(vector3_t v_Gyro, vector3_t v_Accel, vector3_t v_Mag)
{

	// *** QUATERNION!

	if (MODE_X1ORPLUS0 == 1) {
		float temp = v_Gyro.x;
		float temp2 = v_Gyro.y;
		v_Gyro.x = M_SQRT1_2 * temp - M_SQRT1_2 * temp2;
		v_Gyro.y = M_SQRT1_2 * temp2 + M_SQRT1_2 * temp;
	}
	
	Gyro.X.value = (v_Gyro.x - (float)Gyro.X.error)/(float)AHRS_RATE;
	Gyro.Y.value = (v_Gyro.y - (float)Gyro.Y.error)/(float)AHRS_RATE;
	Gyro.Z.value = (v_Gyro.z - (float)Gyro.Z.error)/(float)AHRS_RATE;
	
	quaternionOld.u = quaternion.u;
	quaternionOld.a = quaternion.a;
	quaternionOld.i = quaternion.i;
	quaternionOld.r = quaternion.r;

	quaternion.u -= 0.5*(Gyro.X.value*quaternionOld.a + Gyro.Y.value*quaternionOld.i + Gyro.Z.value*quaternionOld.r);
	quaternion.a += 0.5*(Gyro.X.value*quaternionOld.u + Gyro.Z.value*quaternionOld.i - Gyro.Y.value*quaternionOld.r);
	quaternion.i += 0.5*(Gyro.Y.value*quaternionOld.u - Gyro.Z.value*quaternionOld.a + Gyro.X.value*quaternionOld.r);
	quaternion.r += 0.5*(Gyro.Z.value*quaternionOld.u + Gyro.Y.value*quaternionOld.a - Gyro.X.value*quaternionOld.i);
	
	// precalculated values for optimisation
	float quu = quaternion.u * quaternion.u;
	float qua = quaternion.u * quaternion.a;
	float qui = quaternion.u * quaternion.i;
	// float qur = quaternion.u * quaternion.r; // No gain from precalc
	float qaa = quaternion.a * quaternion.a;
	// float qai = quaternion.a * quaternion.i; // No gain from precalc
	float qar = quaternion.a * quaternion.r;
	float qii = quaternion.i * quaternion.i;
	float qir = quaternion.i * quaternion.r;
	float qrr = quaternion.r * quaternion.r;
	float qaaPqii = qaa + qii;
	float TquiMqar = 2 * (qui - qar);
	float TqirPqua = 2 * (qir + qua);
	float m9 = (quu - qaa - qii + qrr);
	
	// renormalise using Taylor expansion
	// sumsqu = (3-(quu + qaaPqii + qrr))/2;
	// quaternion.u *= sumsqu;
	// quaternion.a *= sumsqu;
	// quaternion.i *= sumsqu;
	// quaternion.r *= sumsqu;
	
	// renormalise using fast inverse square root
	float sumsqu = invSqrt(quu + qaaPqii + qrr);
	quaternion.u *= sumsqu;
	quaternion.a *= sumsqu;
	quaternion.i *= sumsqu;
	quaternion.r *= sumsqu;

	// avoid gimbal lock at singularity points
	if (TquiMqar == 1) {
		psiAngle = 2 * atan2(quaternion.a, quaternion.u);
		thetaAngle = M_PI_2;
		phiAngle = 0;
	}
	else if (TquiMqar == -1) {
		psiAngle = -2 * atan2(quaternion.a, quaternion.u);
		thetaAngle = - M_PI_2;
		phiAngle = 0;
	}
	else {
		thetaAngle = asin(TquiMqar);
		phiAngle = atan2(TqirPqua, (1 - 2*qaaPqii));
		psiAngle = atan2((2*(quaternion.u * quaternion.r + quaternion.a * quaternion.i)), (1 - 2*(qii + qrr)));
	}
	
	if(++ahrsSoftscale > AHRS_CORRECTION) {
		ahrsSoftscale = 0;
		// *** Read Accelerometers
		Accel.X.raw = v_Accel.x;
		Accel.Y.raw = v_Accel.y;
		Accel.Z.raw = v_Accel.z;
		
		sumsqu = invSqrt((float)Accel.X.raw*(float)Accel.X.raw + (float)Accel.Y.raw*(float)Accel.Y.raw + (float)Accel.Z.raw*(float)Accel.Z.raw);	// Accelerometr data is normalised so no need to convert units.
		Accel.X.value = (float)Accel.X.raw * sumsqu;
		Accel.Y.value = (float)Accel.Y.raw * sumsqu;
		Accel.Z.value = (float)Accel.Z.raw * sumsqu;
		
		if (MODE_X1ORPLUS0 == 1) {
			float temp = Accel.X.value;
			float temp2 = Accel.Y.value;
			Accel.X.value = M_SQRT1_2 * temp - M_SQRT1_2 * temp2;
			Accel.Y.value = M_SQRT1_2 * temp2 + M_SQRT1_2 * temp;
		}
		
		// *** Read Magnetometers
		Magneto.X.raw = v_Mag.x;
		Magneto.Y.raw = v_Mag.y;
		Magneto.Z.raw = v_Mag.z;
		
		if (MODE_X1ORPLUS0 == 1) {
			float temp = Magneto.X.raw;
			float temp2 = Magneto.Y.raw;
			Magneto.X.value = M_SQRT1_2 * temp - M_SQRT1_2 * temp2;
			Magneto.Y.value = M_SQRT1_2 * temp2 + M_SQRT1_2 * temp;
		}
		else {
			Magneto.X.value = (float)Magneto.X.raw;
			Magneto.Y.value = (float)Magneto.Y.raw;
		}
		
		Magneto.Z.value = (float)Magneto.Z.raw;
		
		// *** Drift correction
		Gyro.X.verterror = TqirPqua*Accel.Z.value - m9*Accel.Y.value;  // Correction vector
		Gyro.Y.verterror = m9*Accel.X.value + TquiMqar*Accel.Z.value;
		
		float MdotA = Magneto.X.value*Accel.X.value + Magneto.Y.value*Accel.Y.value + Magneto.Z.value*Accel.Z.value;
		float X = (Magneto.X.value - MdotA*Accel.X.value);  // Lagranges Theorum
		float Y = (Magneto.Y.value - MdotA*Accel.Y.value);

		if(Accel.Z.value > 0.3) {
			Gyro.Z.verterror = psiAngle + atan2(Y, X); // Heading correction error calculation
			//if (Accel.Z.value < 0) Gyro.Z.verterror = psiAngle - fatan2(Y, X) + M_PI; // Todo: fix upside down case

			if (Gyro.Z.verterror > M_PI) Gyro.Z.verterror -= M_TWOPI;
			if (Gyro.Z.verterror < -M_PI) Gyro.Z.verterror += M_TWOPI;
		}
		else {
			Gyro.Z.verterror = 0;
		}

		Gyro.X.verterrorInt *= DRIFT_AccelDe;    // Integral decay
		Gyro.Y.verterrorInt *= DRIFT_AccelDe;
		Gyro.Z.verterrorInt *= DRIFT_MagDe;
		Gyro.X.verterrorInt += Gyro.X.verterror;
		Gyro.Y.verterrorInt += Gyro.Y.verterror;
		Gyro.Z.verterrorInt += Gyro.Z.verterror;
		Gyro.X.error = DRIFT_AccelKp * Gyro.X.verterror + DRIFT_AccelKi * Gyro.X.verterrorInt;  // Error is inserted into a PI loop
		Gyro.Y.error = DRIFT_AccelKp * Gyro.Y.verterror + DRIFT_AccelKi * Gyro.Y.verterrorInt;
		Gyro.Z.error = DRIFT_MagKp * Gyro.Z.verterror + DRIFT_MagKi * Gyro.Z.verterrorInt;
	}
	
	if(++ahrsBarometer > AHRS_RATE/200) {    // scale down so Barometer is working at max of 200Hz
		ahrsBarometer = 0;
		//// *** Read Barometer
		//if(Baro.reading < BARO_TEMP_EVERY) {
			//Baro.raw = BaroReadPress();
			//Baro.total -= Baro.history[Baro.count];
			//Baro.total += Baro.raw;
			//Baro.history[Baro.count] = Baro.raw;
			//if(++Baro.count >= BARO_AVERAGING) Baro.count = 0;
		//}
		//else {
			//Baro.temperature = BaroReadTemp();
			//Baro.reading = 0;
		//}
		//
		//if(++Baro.reading < BARO_TEMP_EVERY) BaroStartPress();
		//else BaroStartTemp();
		//
		//Baro.pressure = (float)Baro.total/(float)BARO_AVERAGING;
		//lastAltitude = Baro.altitude;
		//Baro.altitude = BaroApproxAlt(Baro.pressure) + Baro.altitudeOffset;
		//altitude = Baro.altitude;
		
		//if(GPS.fix) {
			//float altitudeError = (float)GPS.altitude - (float)Baro.altitude;
			//Baro.altitudeOffset += altitudeError * DRIFT_Baro;
			////altitude = Baro.altitude * ALT_Bias + GPS.altitude * (1-ALT_Bias);
		//}
	}
	quaternion_t q;
	return q; // fixme return sth
}
