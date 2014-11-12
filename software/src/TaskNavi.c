/*
 * TaskNavi.c
 *
 * Created: 14.10.2013 20:23:55
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

#include <asf.h>
#include <fastmath.h>
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/GPS.h"
#include "modules/vector.h"
#include "modules/NAV.h"
#include "TaskNavi.h"
#include "modules/governing.h"
//#include "menu/menu.h"
//#include "menu/menu_variant.h"

//local prototypes
void TaskNavi(void);



void TaskNavi(void)
{
	OS_WaitTicks(OSALM_NAVIWAIT,500); // wait for serial init
	
	gps_init();
	
	while(1)
	{
		uint8_t evt = OS_WaitEventTimeout(OSEVT_GPSREADY,OSALM_NAVIWAIT,NAVGPSTIMEOUT_ms);
		if(evt != OSEVT_GPSREADY)
		{
			// no GPS!!!
			GPS_timeout(); // just notify the GPS and set Sats to 0
		}
		else
		{
			// work GPS data
			gps_coordinates_t ActPos;
			GPSgetPos(&ActPos); // fixme error ?
			NAV_UpdatePosition_xy(ActPos);

		}
	}
}

