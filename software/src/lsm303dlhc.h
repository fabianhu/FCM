/*
 * lsm303dlhc.h
 *
 * Created: 03.01.2013 18:31:31
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


#ifndef LSM303DLHC_H_
#define LSM303DLHC_H_

#define LSM303_ADDR_MAGNETO 0x1e
#define LSM303_ADDR_ACCELERO 0x19

// reading all result fields: Read from address 27h ff = A7h (first bit set for auto increment)
typedef struct lsm303_acc_result_tag
{
	uint8_t STATUS_REG_A;
	uint8_t OUT_X_L_A;
	uint8_t OUT_X_H_A;
	uint8_t OUT_Y_L_A;
	uint8_t OUT_Y_H_A;
	uint8_t OUT_Z_L_A;
	uint8_t	OUT_Z_H_A;
} lsm303_acc_result_t;


// read magnetic results: Read from address 03h (auto increment always active)
typedef struct lsm303_mag_result_tag
{
	uint8_t OUT_X_H_M;
	uint8_t OUT_X_L_M;
	uint8_t OUT_Z_H_M;
	uint8_t OUT_Z_L_M;
	uint8_t OUT_Y_H_M;
	uint8_t OUT_Y_L_M;
	uint8_t SR_REG_Mg; // 00000010  if read in sequence.. ??
	uint8_t IRA_REG_M; // 01001000
	uint8_t IRB_REG_M; // 00110100
	uint8_t IRC_REG_M; // 00110011

}lsm303_mag_result_t;

void lsm303_config_accel(vfpv_t delay1ms);
void lsm303_config_magnet(vfpv_t delay1ms);
void lsm303_TWI_trig_read_accel(void);
void lsm303_TWI_trig_read_magnet(void);
int32_t lsm303_get_acc_results(vector3_t* res);
int32_t lsm303_get_mag_results(vector3_t* res);



#endif /* LSM303DLHC_H_ */