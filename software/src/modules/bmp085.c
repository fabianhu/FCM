/*  $Date: 2009/10/23 $
 *  $Revision: 1.2 $
 */

/*
* Copyright (C) 2009 Bosch Sensortec GmbH
*
* BMP085 pressure sensor API
* 
* Usage:  Application Programming Interface for BMP085 configuration and data read out
*
* 
* Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in 
  compliance with the License and the following stipulations. The Apache License , Version 2.0 is applicable unless 
  otherwise stated by the stipulations of the disclaimer below. 
 
* You may obtain a copy of the License at 

   http://www.apache.org/licenses/LICENSE-2.0
  
 

Disclaimer 

* Common:
* This Work is developed for the consumer goods industry. It may only be used 
* within the parameters of the respective valid product data sheet.  The Work 
* provided with the express understanding that there is no warranty of fitness for a particular purpose. 
* It is not fit for use in life-sustaining, safety or security sensitive systems or any system or device 
* that may lead to bodily harm or property damage if the system or device malfunctions. In addition, 
* the Work is not fit for use in products which interact with motor vehicle systems.  
* The resale and/or use of the Work are at the purchaser’s own risk and his own responsibility. The 
* examination of fitness for the intended use is the sole responsibility of the Purchaser. 
*
* The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for 
* incidental, or consequential damages, arising from any Work or Derivative Work use not covered by the parameters of 
* the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch 
* Sensortec for all costs in connection with such claims.
*
* The purchaser must monitor the market for the purchased Work and Derivative Works, particularly with regard to 
* product safety and inform Bosch Sensortec without delay of all security relevant incidents.
*
* Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid 
* technical specifications of the product series. They are therefore not intended or fit for resale to third 
* parties or for use in end products. Their sole purpose is internal client testing. The testing of an 
* engineering sample may in no way replace the testing of a product series. Bosch Sensortec 
* assumes no liability for the use of engineering samples. By accepting the engineering samples, the 
* Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering 
* samples.
*
* Special:
* This Work and any related information (hereinafter called "Information") is provided free of charge 
* for the sole purpose to support your application work. The Work and Information is subject to the 
* following terms and conditions: 
*
* The Work is specifically designed for the exclusive use for Bosch Sensortec products by 
* personnel who have special experience and training. Do not use this Work or Derivative Works if you do not have the 
* proper experience or training. Do not use this Work or Derivative Works fot other products than Bosch Sensortec products.  
*
* The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no 
* responsibility for the consequences of use of such Information nor for any infringement of patents or 
* other rights of third parties which may result from its use. No license is granted by implication or 
* otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are 
* subject to change without notice.
*
*/


/*! \file bmp085.c
    \brief This file contains all function implementations for the BMP085 API
    
    Details.
*/

#include <asf.h>
#include "modules/types.h"
#include "modules/twi.h"
#include "bmp085.h"
#include "../FabOS_config.h"
#include "../FabOS/FabOS.h"
#include "modules/emergency.h"

/* API internal helper functions */
static int bmp085_read_cal_param(vfpv_t delay1ms);

uint8_t bmp085_press_raw[3];
uint16_t  bmp085_temp_raw;

bmp085_t* p_bmp085 = 0;                      /**< pointer to BMP085 device area */

/** initialize BMP085

  This function initializes the BMP085 pressure sensor.
  The function automatically detects the sensor type and stores this for all future communication and calculation steps
  \param *bmp085_t pointer to bmp085 device data structure
  \return result of communication routines

*/

int bmp085_init(vfpv_t delay1ms) 
{
	if(delay1ms == NULL)
		return 1;
	
	char comres=0;
	unsigned char data;
	//  long dummy;
	static bmp085_t bmp085; // the calibration values are stored here

	p_bmp085 = &bmp085;                                      /* assign BMP085 ptr */
 	TWI_read_ISR(BMP085_I2C_ADDR, BMP085_CHIP_ID__REG,1,&data);  /* read Chip Id */
	delay1ms();
	
	p_bmp085->chip_id = BMP085_GET_BITSLICE(data, BMP085_CHIP_ID);  
	p_bmp085->number_of_samples = 8;  
	p_bmp085->oversampling_setting=3;

	TWI_read_ISR(BMP085_I2C_ADDR, BMP085_VERSION_REG,1,&data); /* read Version reg */
	delay1ms();
    
	p_bmp085->ml_version = BMP085_GET_BITSLICE(data, BMP085_ML_VERSION);        /* get ML Version */
	p_bmp085->al_version = BMP085_GET_BITSLICE(data, BMP085_AL_VERSION);        /* get AL Version */
	comres = bmp085_read_cal_param(delay1ms); /* readout bmp085 calibparam structure */

	return comres;

}

/** read out parameters cal_param from BMP085 memory
   \return result of communication routines
*/

static int bmp085_read_cal_param(vfpv_t delay1ms) 
{
	if(delay1ms == NULL)
		return 1;
		
	int comres = 0;
	unsigned char data[BMP085_PROM_DATA__LEN];
	TWI_read_ISR(BMP085_I2C_ADDR, BMP085_PROM_START__ADDR, BMP085_PROM_DATA__LEN, data);
	delay1ms();
	delay1ms();
  
	/*parameters AC1-AC6*/
	p_bmp085->cal_param.ac1 =  (data[0] <<8) | data[1];
	p_bmp085->cal_param.ac2 =  (data[2] <<8) | data[3];
	p_bmp085->cal_param.ac3 =  (data[4] <<8) | data[5];
	p_bmp085->cal_param.ac4 =  (data[6] <<8) | data[7];
	p_bmp085->cal_param.ac5 =  (data[8] <<8) | data[9];
	p_bmp085->cal_param.ac6 = (data[10] <<8) | data[11];
  
	/*parameters B1,B2*/
	p_bmp085->cal_param.b1 =  (data[12] <<8) | data[13];
	p_bmp085->cal_param.b2 =  (data[14] <<8) | data[15];
  
	/*parameters MB,MC,MD*/
	p_bmp085->cal_param.mb =  (data[16] <<8) | data[17];
	p_bmp085->cal_param.mc =  (data[18] <<8) | data[19];
	p_bmp085->cal_param.md =  (data[20] <<8) | data[21];
  
  
	uint16_t* ptr = (uint16_t*)&(p_bmp085->cal_param);
  
	// according data sheet no word must be 0 or 0xffff, if comm is OK.
	for(int i = 1;i<BMP085_PROM_DATA__LEN/2;i++)
	{
	if(ptr[i] == 0 || ptr[i] == 0xffff)	 
	{
		emstop(1); // this is during init. hang here, if Baro does not work.
		comres++;
	} 
	}
  
	return comres;  
  
}


/** calculate temperature from ut
  ut was read from the device via I2C and fed into the right calc path for BMP085
  \param ut parameter ut read from device
  \return temperature in steps of 0.1 deg celsius
  \see bmp085_read_ut()
*/

short bmp085_get_temperature(void) 
//short bmp085_get_temperature(unsigned long ut) 
{
	unsigned long ut = bmp085_temp_raw;
  short temperature;
  long x1,x2;// long x3,x4,y2,y3,y4;    

    x1 = (((long) ut - (long) p_bmp085->cal_param.ac6) * (long) p_bmp085->cal_param.ac5) >> 15;
    x2 = ((long) p_bmp085->cal_param.mc << 11) / (x1 + p_bmp085->cal_param.md);
    p_bmp085->param_b5 = x1 + x2;

  temperature = ((p_bmp085->param_b5 + 8) >> 4);  // temperature in 0.1°C

  return (temperature);
}

/** calculate pressure from up
  up was read from the device via I2C and fed into the right calc path for either BMP085
  In case of BMP085 averaging is done through oversampling by the sensor IC

  \param ut parameter ut read from device
  \return temperature in steps of 1.0 Pa
  \see bmp085_read_up()
*/
unsigned long bmp085_read_up(void)
{
	unsigned long up=0;
	up = (((unsigned long) bmp085_press_raw[0] << 16) | ((unsigned long) bmp085_press_raw[1] << 8) | (unsigned long) bmp085_press_raw[2]) >> (8-p_bmp085->oversampling_setting);
	p_bmp085->number_of_samples = 1;
	return up;
}



//long bmp085_get_pressure(void)
long bmp085_calc_pressure(unsigned long up)
{
  
   long pressure,x1,x2,x3,b3,b6;
   unsigned long b4, b7;
   
   b6 = p_bmp085->param_b5 - 4000; // !!! temperature influence
   //*****calculate B3************
   x1 = (b6*b6) >> 12;	 	 
   x1 *= p_bmp085->cal_param.b2;
   x1 >>=11;

   x2 = (p_bmp085->cal_param.ac2*b6);
   x2 >>=11;

   x3 = x1 +x2;

	b3 = (((((long)p_bmp085->cal_param.ac1 )*4 + x3) <<p_bmp085->oversampling_setting) + 2) >> 2;

   //*****calculate B4************
   x1 = (p_bmp085->cal_param.ac3* b6) >> 13;
   x2 = (p_bmp085->cal_param.b1 * ((b6*b6) >> 12) ) >> 16;
   x3 = ((x1 + x2) + 2) >> 2;
   b4 = (p_bmp085->cal_param.ac4 * (unsigned long) (x3 + 32768)) >> 15;
     
   b7 = ((unsigned long)(up - b3) * (50000>>p_bmp085->oversampling_setting));   
   if (b7 < 0x80000000)
   {
     pressure = (b7 << 1) / b4;
   }
   else
   { 
     pressure = (b7 / b4) << 1;
   }
   
   x1 = pressure >> 8;
   x1 *= x1;
   x1 = (x1 * BMP085_PARAM_MG) >> 16;
   x2 = (pressure * BMP085_PARAM_MH) >> 16;
   pressure += (x1 + x2 + BMP085_PARAM_MI) >> 4;	// pressure in Pa  

   return (pressure);
}


/** read out ut for temperature conversion
   \return ut parameter that represents the uncompensated temperature sensors conversion value
*/

/*unsigned short bmp085_read_ut ()
//unsigned short bmp085_get_ut ()
{
	unsigned short ut;
	unsigned char data[2];
	unsigned char ctrl_reg_data;
	int wait_time;

    ctrl_reg_data = BMP085_T_MEASURE;
    wait_time = BMP085_TEMP_CONVERSION_TIME;

	TWI_write_nowait(BMP085_I2C_ADDR, BMP085_CTRL_MEAS_REG, ctrl_reg_data); // takes 0.025ms per byte = 0.075ms
  
	OS_WaitTicks(OSALM_TWIWAIT, BMP085_TEMP_CONVERSION_TIME);
	
	TWI_read_ISR(BMP085_I2C_ADDR, BMP085_ADC_OUT_MSB_REG, 2, data); // takes 0.025ms per byte = 0.1ms
	TWI_waitISRBusy(); 
	ut = (data[0] <<8) | data[1];
	return (ut);
}*/

uint8_t CommBuf[2]; // static communication buffer

uint32_t bmp085_TWI_trig_temp_meas (void) // return required wait time (5ms)
{
	CommBuf[0] = BMP085_T_MEASURE;

	TWI_write_ISR(BMP085_I2C_ADDR, BMP085_CTRL_MEAS_REG, 1, CommBuf); // takes 0.025ms per byte = 0.075ms
	
	return BMP085_TEMP_CONVERSION_TIME;
	
}



void bmp085_TWI_trig_temp_read(void) // takes 0.1ms to complete (async!!!!)
{
	TWI_read_ISR(BMP085_I2C_ADDR, BMP085_ADC_OUT_MSB_REG, 2, (uint8_t*)&bmp085_temp_raw); // takes 0.025ms per byte = 0.1ms
}



/** read out up for pressure conversion
  depending on the oversampling ratio setting up can be 16 to 19 bit
   \return up parameter that represents the uncompensated pressure value
*/

/*unsigned long bmp085_read_up (void)
//unsigned long bmp085_get_up ()
{
	//  int i;
	unsigned long up=0;
	unsigned char data[3];    
	unsigned char ctrl_reg_data;
  
	ctrl_reg_data = BMP085_P_MEASURE + (p_bmp085->oversampling_setting << 6);
	TWI_write_nowait(BMP085_I2C_ADDR, BMP085_CTRL_MEAS_REG, ctrl_reg_data);
	
	int ms =  2 + (3 << (p_bmp085->oversampling_setting) ) ;
	OS_WaitTicks(OSALM_TWIWAIT,ms);
	TWI_read_ISR(BMP085_I2C_ADDR, BMP085_ADC_OUT_MSB_REG, 3, data);
	TWI_waitISRBusy();
	up = (((unsigned long) data[0] << 16) | ((unsigned long) data[1] << 8) | (unsigned long) data[2]) >> (8-p_bmp085->oversampling_setting);
	p_bmp085->number_of_samples = 1;

	return (up);
}*/



uint32_t bmp085_TWI_trig_press_meas(void) // return wait time
{
	
	CommBuf[0] = BMP085_P_MEASURE + (p_bmp085->oversampling_setting << 6);
 	TWI_write_ISR(BMP085_I2C_ADDR, BMP085_CTRL_MEAS_REG,1, CommBuf);
	
	int ms =  2 + (3 << (p_bmp085->oversampling_setting) ) ;
	return ms;
}



void bmp085_TWI_trig_press_read(void)
{
	TWI_read_ISR(BMP085_I2C_ADDR, BMP085_ADC_OUT_MSB_REG, 3, bmp085_press_raw); // this is 5 bytes over TWI... at 400kHz 40kHzpr byte; 8kHz -> 0,125ms
}

int32_t bmp085_calcHeight_mm(int32_t pBaroPas) //return h in mm, linearized formula, valid from 0..2000m NN
{
	return ((pBaroPas*-9157)/100)+(9254*1000);
}