/*
 * governing.c
 *
 * Created: 07.12.2012 18:27:13
 *
 * (c) 2012-2014 by Fabian Huslik
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
#include "math.h"
#include "../FabOS_config.h"
#include "../FabOS/FabOS.h"
#include "governing.h"
#include "../config.h"
#include "testsuite.h"



int32_t PID(pid_t* inst, int32_t actual, int32_t set, int32_t kP, int32_t kI, int32_t kD, int32_t _lowerLimit, int32_t _upperLimit)
{
	// use powers of 2 for shift-divide.
	#define I_REDUCTION 128L
	#define P_REDUCTION 16L
	#define D_REDUCTION 2L

	int32_t res;
	int32_t P, D;
	int32_t diff;
	int32_t der;

	diff = set-actual;

	der = inst->old - actual ;
	inst->old = actual;

	P = (kP * diff)/P_REDUCTION ;
	inst->I = inst->I + (kI*diff);
	inst->I = limit(inst->I, _lowerLimit*I_REDUCTION, _upperLimit*I_REDUCTION); // wind-up protection
	D = (kD * der)/D_REDUCTION;

	res = P + (inst->I/I_REDUCTION) + D;

	return res ;

}

// standard PID controller with internal derivative
float PIDf(pidf_t* inst, float actual, float set, float kP, float kI, float kD, float lowerLimit, float upperLimit)
{
	volatile float res;
	volatile float P, D;
	volatile float diff;
	volatile float der;

	diff = set-actual;

	der = diff - inst->old_diff ;
	inst->old_diff = diff;

	P = kP * diff;
	inst->I = inst->I + (kI*diff);
	inst->I = limitf(inst->I, lowerLimit, upperLimit); // wind-up protection
	D = kD * der;

	res = P + inst->I + D;

	return res ;

}

// standard PID controller with external derivative
float PIDfxD(pidf_t* inst, float actual, float set, float kP, float kI, float kD, float lowerLimit, float upperLimit, float speed)
{
	volatile float res;
	volatile float P, D;
	volatile float diff;

	diff = set-actual;

	P = kP * diff;
	inst->I = inst->I + (kI*diff);
	inst->I = limitf(inst->I, lowerLimit, upperLimit); // wind-up protection
	D = kD * speed;

	res = P + inst->I + D;

	return res ;

}


int32_t propscale(int32_t _value, int32_t _minin, int32_t _maxin, int32_t _minout, int32_t _maxout)
{
	int32_t rangedvalue;
	int32_t inrange = _maxin-_minin;
	int32_t outrange = _maxout-_minout;
	if(inrange== 0) return 0; // divide by zero protection
	
	_value = limit(_value,_minin,_maxin);
	
	rangedvalue = (_value-_minin)*outrange/inrange;

	return rangedvalue + _minout;
}

// filter with external "memory".
// _memVal will be _newVal*_base to increase resolution; multiply by _base for filter initialisation!
int32_t Filter_mem(int32_t* _memVal, int32_t _newVal, int32_t _base)
{
	*_memVal = *_memVal - (*_memVal/_base) + _newVal;
	
	return *_memVal / _base;
}

float Filter_f(float _oldVal, float _newVal, float _factor)
{
	return _oldVal * (1.0 - _factor) + _newVal * _factor ;
}



float limitf(float val, float low, float upp)
{
	volatile float ret; // without there come false results.
	if(val > upp)
	{
		ret = upp;
	}
	else
	if(val < low)
	{
		ret = low;
	}
	else
	{
		ret = val;
	}
	return ret;
	
}


/*
Browns linear exponent filter
taken from http://www.camelsoftware.com/firetail/blog/uncategorized/scalar-low-pass-filters-a-comparison-of-algorithms/
the code was c-ified from the c++ example

The simple expo filter works well over time to find the average of the readings, but what if the average is trending? 
What if the average is climbing?? In this situation the output from a simple expo filter will begin to lag behind the true value. This is the biggest weakness of the simple expo filter.

Mr. Brown thought about this and what he came up with is called the Linear Exponential filter. 
This filter keeps track of two smoothed values. A singly smoothed value, calculated in the same way as the simple expo filter, and a doubly smoothed value. 
The smoothed value is then smoothed again to create the doubly smoothed value. The singly smoothed value and the doubly smoothed value are then amalgamated to form an estimate of the true value.

The linear expo filter is able to respond to linear changes of the average value very well. This means there is less ‘lag’ when the input rises and falls.

This filter also has only one setting, alpha, which controls the responsiveness of the filter.
*/
float BrownLinearExpo(brownfilter_t* inst, float measurement, float alfa)
{
    //alfa standard = 0.5;
	
	inst->filtered1 = alfa * measurement + (1 - alfa) * inst->filtered1; // filter once
	inst->filtered2 = alfa * inst->filtered1 + (1 - alfa) * inst->filtered2; // filter twice

	float mx = (alfa / (1-alfa) )*(inst->filtered1 - inst->filtered2);
	float t = (2*inst->filtered1 - inst->filtered2);
	
	return mx + t; 
}


#if TEST_RUN == 1
void test_PIDf()
{
	// the PID with limitations
	
	volatile float ret;
	static pidf_t pidinst;
	pidinst.I = 0;
	pidinst.old_diff = 0;
	ret = PIDf(&pidinst,0,1,1,0,0,-5,5);
	assert(fabs(ret-1) < epsilon);
	ret = PIDf(&pidinst,0,10,1,0,0,-5,5); // only P
	assert(fabs(ret-10) < epsilon); // P is not limited
	assertfequal(ret,10);
	assertfequal(pidinst.I,0);
	ret = PIDf(&pidinst,0,-10,0,1,0,-5,5); // only I with limit
	assert(fabs(ret+5) < epsilon);
	assertfequal(ret,-5);
	
	pidinst.I = 0; // clear I
	pidinst.old_diff = 0;
	ret = PIDf(&pidinst,0,1,1,0,0,-5,5);
	assert(fabs(ret-1) < epsilon);
	
}
#endif



