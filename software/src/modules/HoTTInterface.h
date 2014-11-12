/*
 * UartHoTT.h
 *
 * Created: 02.11.2012 23:22:06
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


#ifndef UARTHOTT_H_
#define UARTHOTT_H_

void HoTT_Usart_init( void );
void HoTTSendByte( uint8_t c );
uint8_t HoTTRecByte( void );
void HoTTEnableReceiverMode( void );
void HoTTEnableTransmitterMode( void );
uint8_t HoTTGetAvailable( void );
void HoTTFlushBuffer( void );
void HoTT_byte_handler( uint8_t c );

#endif /* UARTHOTT_H_ */