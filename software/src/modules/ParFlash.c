/*
 * ParFlash.c
 *
 * Created: 31.10.2012 11:37:55
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

#include <asf.h>
#include <string.h>
#include "ParFlash.h"
#include "CRC_CCITT.h"
#include "emergency.h"


typedef struct ParHeader_tag
{
	uint16_t version;
	uint16_t crc;
	uint32_t len;
}ParHeader_t;


int ParFlash_load(uint8_t* data, uint32_t len, uint16_t version) // return 0 on OK
{
	
	
	int ret = 0;
	uint16_t loadCRC;
	ParHeader_t* p_hdr = (ParHeader_t*)AVR32_FLASHC_USER_PAGE_ADDRESS;
	uint8_t* p_data = (uint8_t*)AVR32_FLASHC_USER_PAGE_ADDRESS+sizeof(ParHeader_t);
	
	if(version == p_hdr->version) 
	{
		if(len == p_hdr->len) 
		{	
			loadCRC = calc_crc_ccitt(p_data,len);
			if(loadCRC == p_hdr->crc) 
			{
				memcpy(data,p_data,len);
			}
			else
			{
				ret = 3;
			}
		}
		else
		{
			ret = 2;
		}
	}
	else
	{
		ret = 1;
	}
	
	if(ret != 0)
		flashc_erase_user_page(true);
	
	return ret;
}


void ParFlash_save(uint8_t* data, uint32_t len, uint16_t version)
{
	ParHeader_t hdr;
	
	if ((len + sizeof(ParHeader_t)) > AVR32_FLASHC_USER_PAGE_SIZE) 
	{
		emstop(7);
		return;
	}
	
	hdr.crc = calc_crc_ccitt(data,len);
	hdr.len = len;
	hdr.version = version;
	
	//erase the page
	if(!flashc_erase_user_page(true)) // hang at flash erase failure
		emstop(6);
	
	// write header;
	flashc_memcpy(AVR32_FLASHC_USER_PAGE,&hdr,sizeof(ParHeader_t),false);
	
	// data
	flashc_memcpy(AVR32_FLASHC_USER_PAGE+sizeof(ParHeader_t),data,len,false); // that should be all and it should survive chip erase.

}




