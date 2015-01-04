/*
 * lsm303dlhc.c
 *
 * Created: 03.01.2013 18:31:16
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

#include <asf.h>
#include "modules/types.h"
#include "modules/vector.h"
#include "TaskTWI.h"
#include "lsm303dlhc.h"
#include "modules/twi.h"

int32_t lsm303_get_mag_results_raw(vector3_t* res); // return in nT

uint8_t LSM303_CTRL_REG20h[] = {
	0b10010111, // CTRL_REG1_A ODR3       ODR2    ODR1    ODR0     LPen     Zen      Yen        Xen		// 1344Hz 
	0b10000000, // CTRL_REG2_A HPM1       HPM0    HPCF2   HPCF1    FDS      HPCLICK  HPIS2      HPIS1	// normal mode, no filter // 
	0b00010000, // CTRL_REG3_A I1_CLICK   I1_AOI1 I1_AOI2 I1_DRDY1 I1_DRDY2 I1_WTM   I1_OVERRUN --
	0b00011000, // CTRL_REG4_A BDU        BLE     FS1     FS0      HR       0        0          SIM		// +-4G / High res on. ! Watch scaling in TaskTWI.c !
	0b01000000, // CTRL_REG5_A BOOT       FIFO_EN --      --       LIR_INT1 D4D_INT1 LIR_INT2   D4D_INT2
	0b00000000, // CTRL_REG6_A I2_CLICKen I2_INT1 I2_INT2 BOOT_I1  P2_ACT   --       H_LACTIVE  --
	0b00000000, // REFERENCE
};
// FIFO_CTRL_REG_A (2Eh) and all further: default

uint8_t LSM303_INT1_SRC_A_31h[] = {
	0b00000000
};	

uint8_t LSM303_FIFO_CTRL_REG_A_2Eh[] = {
	0b01000001 // 
};

uint8_t LSM303_CTRL_MAG_00h[] = {
	0b10011100, // CRA_REG_M (00h) // 220Hz // default would be 15Hz
	0b00100000, // CRB_REG_M (01h) // X,Y: 1100 bit/Gauss ; Z 980 bit/Gauss
	0b00000000  // MR_REG_M (02h) // continuous conversion mode
};

lsm303_acc_result_t acc_twi_buf;
lsm303_mag_result_t mag_twi_buf;


void lsm303_config_accel(vfpv_t delay1ms)
{
	if(delay1ms == NULL)
		return;

	TWI_write_ISR(LSM303_ADDR_ACCELERO,0x20|0x80,7,LSM303_CTRL_REG20h); // MSB on for auto increment..
	delay1ms();
	TWI_write_ISR(LSM303_ADDR_ACCELERO,0x31,7,LSM303_INT1_SRC_A_31h);
	delay1ms();
}


void lsm303_config_magnet(vfpv_t delay1ms)
{
	if(delay1ms == NULL)
		return;
	
	TWI_write_ISR(LSM303_ADDR_MAGNETO,0x00,3,LSM303_CTRL_MAG_00h);
	delay1ms();
}


void lsm303_TWI_trig_read_accel(void)
{
	TWI_read_ISR(LSM303_ADDR_ACCELERO,0x27|0x80,7,(uint8_t*)&acc_twi_buf);
}


void lsm303_TWI_trig_read_magnet(void)
{
	TWI_read_ISR(LSM303_ADDR_MAGNETO,0x03,7+3,(uint8_t*)&mag_twi_buf);
}


int32_t lsm303_get_acc_results(vector3_t* res) // results in mg
{
	int16_t xr,yr,zr;
	int32_t state;
	
	xr = acc_twi_buf.OUT_X_L_A | (acc_twi_buf.OUT_X_H_A<<8);
	yr = acc_twi_buf.OUT_Y_L_A | (acc_twi_buf.OUT_Y_H_A<<8);
	zr = acc_twi_buf.OUT_Z_L_A | (acc_twi_buf.OUT_Z_H_A<<8);
	
	if((acc_twi_buf.STATUS_REG_A & 0x0f) == 0x0f)
	{
		res->x = -xr;
		res->y = yr;
		res->z = -zr;
		state = 0;
	}
	else
	{	
		/*res->x = 0; // ignore false (empty) reads !
		res->y = 0;
		res->z = 0;*/
		state = 1;
	}		
	return state;
}



/*
Am Äquator hat das Magnetfeld eine Stärke von ca. 30 µT = 30.000 nT. 
An den Polen ist der Betrag doppelt so groß. 
In Mitteleuropa sind es ca. 48 µT, wobei ca. 20 µT in der horizontalen und ca. 44 µT in der vertikalen Richtung auftreten.

1µT = 10mG

Zu erwartende Stärke in Europa:
48µT = 480mG = 0,48G

*/

int32_t lsm303_get_mag_results(vector3_t* res) // return in nT
{
	vector3_t r;
	int32_t ret = lsm303_get_mag_results_raw(&r);
	res->x = r.x;
	res->y = r.y;
	res->z = r.z;

	return ret;
}


int32_t lsm303_get_mag_results_raw(vector3_t* res) // return in nT
{
	int32_t state;
	int16_t xr,yr,zr;
	
	xr = mag_twi_buf.OUT_X_L_M | (mag_twi_buf.OUT_X_H_M<<8);
	yr = mag_twi_buf.OUT_Y_L_M | (mag_twi_buf.OUT_Y_H_M<<8);
	zr = mag_twi_buf.OUT_Z_L_M | (mag_twi_buf.OUT_Z_H_M<<8);
	
	if(1/*mag_twi_buf.IRA_REG_M == 0b01001000*/) // fixme check --> not possible to read the SR before the data, because aligned backwards.
	{
		res->x = -xr*9091/100; // 1100 LSB/GaussXY   -> 0,90909 mGauss / LSB -> 90,91 nT / LSB
		res->y = yr*9091/100;
		res->z = -zr*10204/100; // 980 LSB/GaussZ  ->  1,0204 mGauss / LSB -> 102,04 nT / LSB
		state = 0;
	}
	else
	{
		res->x = 0;
		res->y = 0;
		res->z = 0;
		state = 1;
		asm("breakpoint"); // fixme how to proceed? -> error handling for TWI failure ?
	}
	return state;	
}

