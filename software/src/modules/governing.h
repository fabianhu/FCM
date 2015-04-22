/*
 * governing.h
 *
 * Created: 07.12.2012 18:27:30
 *
 * (c) 2012-2015 by Fabian Huslik
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


#ifndef GOVERNING_H_
#define GOVERNING_H_

/*#define min(x,y) (((x)<(y))?(x):(y))
#define max(x,y) (((x)>(y))?(x):(y))*/
#define limit(v,l,u) (((v)<(l))?(l):(((v)>(u))?(u):(v)))

float limitf(float val, float low, float upp);

typedef struct pid_tag
{
	int32_t I;
	int32_t old; // for derivative calc.
}pid_t;

typedef struct pidf_tag
{
	float I;
	float old_diff; // for derivative calc.
}pidf_t;

typedef struct brownfilter_tag
{
	float filtered2;
	float filtered1;
}brownfilter_t;

int32_t PID(pid_t* inst, int32_t actual, int32_t set, int32_t kP, int32_t kI, int32_t kD, int32_t lowerLimit, int32_t upperLimit);
float PIDf(pidf_t* inst, float actual, float set, float kP, float kI, float kD, float lowerLimit, float upperLimit);
float PIDfxD(pidf_t* inst, float actual, float set, float kP, float kI, float kD, float lowerLimit, float upperLimit, float speed);

int32_t propscale(int32_t _value, int32_t _minin, int32_t _maxin, int32_t _minout, int32_t _maxout);
int32_t plateau(int32_t pVal);
int32_t Filter_mem(int32_t* _memVal, int32_t _newVal, int32_t _base);
float Filter_f(float _oldVal, float _newVal, float _factor); // float variant
float BrownLinearExpo(brownfilter_t* inst, float measurement, float alfa);

void test_PIDf(void);

#endif /* GOVERNING_H_ */