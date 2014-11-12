/*
 * ParFlash.h
 *
 * Created: 31.10.2012 11:38:05
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


#ifndef PARFLASH_H_
#define PARFLASH_H_

int ParFlash_load(uint8_t* data, uint32_t len, uint16_t version); // return 0 on OK

void ParFlash_save(uint8_t* data, uint32_t len, uint16_t version);

#endif /* PARFLASH_H_ */