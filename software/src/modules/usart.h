/*
 * usart.h
 *
 * Created: 12.02.2014 22:03:46
 *
 * (c) 2014-2015 by Fabian Huslik
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


#ifndef USART_H_
#define USART_H_


void USART_init(uint32_t number, uint32_t baudrate , uint8_t* RXbuffer, uint32_t RXBufferSize, vfpuc_t RX_byte_handler, vfpv_t RX_block_handler); // initialize the usart

uint8_t* USART_GetRxData(int ch, uint32_t* len); // get the data inside of the RX_frame_ISR handler or can be used as poll.
void USART_Send(int ch, uint8_t* stuff,uint32_t len); // send some bytes. This is done ISR based.
void USART_putchar(int ch, uint8_t c);

#endif /* USART_H_ */