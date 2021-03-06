/*
 * version.h
 *
 * Created: 23.09.2013 01:05:31
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

#ifndef VERSION_H_
#define VERSION_H_

#define VERSION_MAJOR 100	// 0..255 100+ = Bootloader
#define VERSION_MINOR 2		// 0..255
#define VERSION_BUILD 2		// 0..65535
#define VERSION_BUILDL (VERSION_BUILD & 0xff)
#define VERSION_BUILDH ((VERSION_BUILD & 0xff00) >> 8)

// magic macro appending stuff
#define STR(x)   #x
#define XSTR(x)  STR(x)
#define VERSION_STRING XSTR(VERSION_MAJOR)"." XSTR(VERSION_MINOR)"." XSTR(VERSION_BUILD)
#define VERSION_SHORT  XSTR(VERSION_MAJOR)"." XSTR(VERSION_MINOR)



#endif /* VERSION_H_ */