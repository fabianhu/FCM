/*
 * servo_out.h
 *
 * Created: 06.01.2012 14:18:19
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


#ifndef SERVO_OUT_H_
#define SERVO_OUT_H_

typedef struct motor_tag
{
	int32_t sens_x;
	int32_t sens_y;
	int32_t sens_z;
	int32_t sens_a;
}motor_t;

#define NUMOUTPUTS 6 // leave as it is for this HW!

extern void servo_out_init(void);
extern void servo_out_set(uint8_t n, uint32_t val); // defined as 0..SERVO_SCALE (10000)
void MixSetOut(int32_t ox, int32_t oy, int32_t oz, int32_t oa);
void DOPgmESCs(void);
void servo_out_zero(void);


#endif /* SERVO_OUT_H_ */