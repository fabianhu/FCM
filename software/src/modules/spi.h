/*
 * spi.h
 *
 * Created: 21.02.2012 20:55:07
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


#ifndef SPI_H_
#define SPI_H_

#define CHIPSEL_L3GD20 0

#define SPI_QLEN 12


typedef union
{
  unsigned long                 reg;
  avr32_spi_tdr_t               b;
} u_avr32_spi_tdr_t;


extern uint16_t SPI_resQ[SPI_QLEN];

void SPI_init(void);
uint32_t SPI_AddQ16(uint16_t data, uint8_t cs, uint8_t last);
uint32_t SPI_AddQ8(uint8_t data, uint8_t cs, uint8_t last);
void SPI_startqueue(void);
void SPI_ResetQ(void);
void SPI_EndQ(void);
void SPI_RegisterReadyHandler(vfpv_t fp);
void SPI_ReInit(void);


#endif /* SPI_H_ */