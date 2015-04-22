
/*
 * version.h
 *
 * Created: 14.05.2014 20:47:53
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

#ifndef VERSION_H_
#define VERSION_H_

#include "version-auto.h"

#define VERSION_MAJOR 2 // 0..255 | 100+ = Bootloader
#define VERSION_MINOR 1 // 0..255
#define VERSION_BUILD VERSION_GIT

// FIXME the build number is 32 bit now!

#define VERSION_BUILDL (VERSION_BUILD & 0xff)
#define VERSION_BUILDH ((VERSION_BUILD & 0xff00) >> 8)

#define PARAMETERVERSION 24 // if format is changed, checksum will not match.

// magic macro appending stuff
#define STR(x)   #x
#define XSTR(x)  STR(x)
#if 0 == 1 // modded ?
	#define VERSION_STRING XSTR(VERSION_MAJOR)"." XSTR(VERSION_MINOR)"." XSTR(VERSION_BUILD)"B"
#else
	#define VERSION_STRING XSTR(VERSION_MAJOR)"." XSTR(VERSION_MINOR)"." XSTR(VERSION_BUILD)
#endif
#define VERSION_SHORT  XSTR(VERSION_MAJOR)"." XSTR(VERSION_MINOR)



#endif /* VERSION_H_ */