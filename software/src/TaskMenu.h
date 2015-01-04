/*
 * TaskMenu.h
 *
 * Created: 26.05.2013 21:26:38
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


#ifndef TASKMENU_H_
#define TASKMENU_H_

/*typedef enum
{
	
} eTMNU_State_t;*/

void TMNU_SetPtr(uint8_t* ptr);

void TMNU_TriggerMenu(uint32_t trigger,uint8_t SendBT);




#endif /* TASKMENU_H_ */