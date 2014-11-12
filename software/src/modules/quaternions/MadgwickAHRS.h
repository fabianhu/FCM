//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
// "Open-source resources available on this website are provided under the GNU General Public Licence unless an alternative licence is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	100.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain (org 0.1)
#define betaFast	5.0f

//---------------------------------------------------------------------------------------------------
// Function declarations

quaternion_t MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSSetBetaFast(void);
void MadgwickAHRSSetBetaNormal(void);
bool MadgwickStart(void);

void debugResetMadgwick(void); // debug remove
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
