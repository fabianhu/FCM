/*
 * TaskTWI.h
 *
 * Created: 31.01.2013 10:33:04
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


#ifndef TASKTWI_H_
#define TASKTWI_H_


int32_t twi_getAccel_flt(vector3_t* res);
int32_t twi_getMagnet_flt(vector3_t* res);
int32_t twi_get_h_mm(void);
int32_t twi_get_temp(void);


#endif /* TASKTWI_H_ */