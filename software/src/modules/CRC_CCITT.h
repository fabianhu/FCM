/*
 * CRCCCITT.h
 * Calculate a CRC16 with table(fast)
 * Created: 31.10.2012 11:50:40
 * 
 
 (c) Lammert Bies
 
http://www.lammertbies.nl/comm/software/index.html
believed to be GPL-V3 compatible, as it is offered as "free download" with the following disclaimer:
"The software on this page is believed to function as described in the document present in each archive file. All archive files are in ZIP format.
Use this software at your own risk. Lammert Bies can not be held responsible for any damage, financial loss or injuries resulting from using the software found here. 
If such problems are encountered using this software, please send me a mail message describing the problem and the software will either be updated, or removed from this website. "
 */ 


#ifndef CRC_CCITT_H_
#define CRC_CCITT_H_

// FIRST init the table!
// do never forget to init the table!
void init_crcccitt_tab( void ); 

// incremental / set to 0 first
uint16_t update_crc_ccitt( uint16_t crc, uint8_t c );

// at once
uint16_t calc_crc_ccitt( volatile uint8_t* buf, int len ) ;



#endif /* CRCCCITT_H_ */