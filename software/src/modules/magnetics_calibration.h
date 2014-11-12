/*
 * magnetics_calibration.h
 *
 * Created: 16.03.2013 22:48:20
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


#ifndef MAGNETICS_CALIBRATION_H_
#define MAGNETICS_CALIBRATION_H_


vector3_t magcal_compensate(vector3_t* pMagRaw, vector3_t* magOffset,bool DoCal);

extern vector3_t magcal_debug_vector; // debug only

#endif /* MAGNETICS_CALIBRATION_H_ */