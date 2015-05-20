/*
 * TaskNavi.h
 *
 * Created: 12.11.2013 22:27:59
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


#ifndef TASKNAVI_H_
#define TASKNAVI_H_


void NAVi_GetCommand(vector3_t* cmd); // Get the command from naviagtion task
void NAVi_SetTarget(gps_coordinates_t trg); // Set the target for the navigation task
void Navi_ReSetLastFrameTime(void); // fixme get rid of it, use time from gps frame.
void NAVi_NotifyClosedLoop(bool act); // notify the navi task, if the output is actually used (closed Loop)

#endif /* TASKNAVI_H_ */