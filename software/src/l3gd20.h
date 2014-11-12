/*
 * l3gd20.h
 *
 * Created: 26.02.2012 09:50:05
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
 
void l3gd20_Enable_ISR_transfer(void);
int l3gd20_getValues_raw(int32_t* x, int32_t* y, int32_t* z);
void l3gd20_ConfigInitQueue(void);
void l3gd20_ConfigReadQueue(void);
void l3gd20_CheckWhoAmI(void);
float l3gd20_raw_to_rad(int32_t raw);

// Register Addresses
#define	L3GD20_WHO_AM_I			0x0F	//r		11010100
#define	L3GD20_CTRL_REG1		0x20	//rw	00000111
#define	L3GD20_CTRL_REG2		0x21	//rw	00000000
#define	L3GD20_CTRL_REG3		0x22	//rw	00000000
#define	L3GD20_CTRL_REG4		0x23	//rw	00000000
#define	L3GD20_CTRL_REG5		0x24	//rw	00000000
#define	L3GD20_REFERENCE		0x25	//rw	00000000
#define	L3GD20_OUT_TEMP			0x26	//r		output
#define	L3GD20_STATUS_REG		0x27	//r		output
#define	L3GD20_OUT_X_L			0x28	//r		output
#define	L3GD20_OUT_X_H			0x29	//r		output
#define	L3GD20_OUT_Y_L			0x2A	//r		output
#define	L3GD20_OUT_Y_H			0x2B	//r		output
#define	L3GD20_OUT_Z_L			0x2C	//r		output
#define	L3GD20_OUT_Z_H			0x2D	//r		output
#define	L3GD20_FIFO_CTR0xL_REG	0x2E	//rw	00000000
#define	L3GD20_FIFO_SRC_REG		0x2F	//r		output
#define	L3GD20_INT1_CFG			0x30	//rw	00000000
#define	L3GD20_INT1_SRC			0x31	//r		output
#define	L3GD20_INT1_TSH_XH		0x32	//rw	00000000
#define	L3GD20_INT1_TSH_XL		0x33	//rw	00000000
#define	L3GD20_INT1_TSH_YH		0x34	//rw	00000000
#define	L3GD20_INT1_TSH_YL		0x35	//rw	00000000
#define	L3GD20_INT1_TSH_ZH		0x36	//rw	00000000
#define	L3GD20_INT1_TSH_ZL		0x37	//rw	00000000
#define	L3GD20_INT1_DURATION	0x38	//rw	00000000

// Default settings
/*
#define	L3GD20_CTRL_REG1_SET		0x11111111 // |DR1|DR0|BW1|BW0|PD|Zen|Yen|Xen|(760Hz with BW 100hz(0b11111111)) 0xFF
#define	L3GD20_CTRL_REG2_SET		0b00110100 // |0|0|HPM1|HPM1|HPCF3|HPCF2|HPCF1|HPCF0|			0x34 = 0b 0011 0100
#define	L3GD20_CTRL_REG3_SET		0b00000000 // |I1_Int1|I1_Boot|H_Lactive|PP_OD|I2_DRDY|I2_WTM|I2_ORun|I2_Empty|		
#define	L3GD20_CTRL_REG4_SET		0b00100000 // |BDU|BLE|FS1|FS0|-|ST1|ST0|SIM|			0xA0 = 0b10100000
#define	L3GD20_CTRL_REG5_SET		0b00010011 // |BOOT|FIFO_EN|--|HPen|INT1_Sel1|INT1_Sel0|Out_Sel1|Out_Sel0|		0x13 = 0b00010011
*/
#define	L3GD20_CTRL_REG1_SET		0b11111111 // |DR1|DR0|BW1|BW0|PD|Zen|Yen|Xen|(760Hz with BW 100hz(0b11111111)) 0xFF
#define	L3GD20_CTRL_REG2_SET		0b00000000 // |0|0|HPM1|HPM1|HPCF3|HPCF2|HPCF1|HPCF0|			0x34 = 0b 0011 0100
#define	L3GD20_CTRL_REG3_SET		0b00001000 // |I1_Int1|I1_Boot|H_Lactive|PP_OD|I2_DRDY|I2_WTM|I2_ORun|I2_Empty|		
#define	L3GD20_CTRL_REG4_SET		0b00100000 // |BDU|BLE|FS1|FS0|-|ST1|ST0|SIM|	2000 deg/s !!!
#define	L3GD20_CTRL_REG5_SET		0b00000000 // |BOOT|FIFO_EN|--|HPen|INT1_Sel1|INT1_Sel0|Out_Sel1|Out_Sel0|		0x13 = 0b00010011


#define	L3GD20_REFERENCE_SET		0b00000000
#define	L3GD20_FIFO_CTR0xL_REG_SET	0b00000000
#define	L3GD20_INT1_CFG_SET			0b00000000
#define	L3GD20_INT1_TSH_XH_SET		0b00000000
#define	L3GD20_INT1_TSH_XL_SET		0b00000000
#define	L3GD20_INT1_TSH_YH_SET		0b00000000
#define	L3GD20_INT1_TSH_YL_SET		0b00000000
#define	L3GD20_INT1_TSH_ZH_SET		0b00000000
#define	L3GD20_INT1_TSH_ZL_SET		0b00000000
#define	L3GD20_INT1_DURATION_SET	0b00000000


typedef struct gyro_tag
{
	int32_t x;
	int32_t y;
	int32_t z;
	int32_t state;
}gyro_t;