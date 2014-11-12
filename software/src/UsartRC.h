/*
 * UsartRC.c
 *
 * Created: 28.07.2012 14:23:05
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

#ifndef MYUART_H_
#define MYUART_H_

void UsartRC_init( void ); // SUMD-RX

uint32_t RC_GetTimeSinceLastRC(void);
void RC_ResetTimeSinceLastRC(void);

#endif /* MYUART.H_H_ */
