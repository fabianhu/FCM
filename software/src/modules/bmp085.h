/*  $Date: 2009/10/23 $
 *  $Revision: 1.2 $
 */

/** \mainpage BMP085 Pressure Sensor API
 * Copyright (C) 2009 Bosch Sensortec GmbH
 *  \section intro_sec Introduction
 * BMP085 digital Altimeter Programming Interface
 * The BMP085 API enables quick access to Bosch Sensortec's digital altimeter.
 * The only mandatory steps are: 
 *
 * 1. linking the target application's communication functions to the API (\ref BMP085_WR_FUNC_PTR, \ref BMP085_RD_FUNC_PTR)
 *
 * 2. calling the bmp085_init() routine, which initializes all necessary data structures for using all functions
 *
 *
 * 
 * \section disclaimer_sec Disclaimer
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
* for the sole purpose to support your application work. The Woek and Information is subject to the 
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

 /** \file bmp085.h
    \brief Header file for all #define constants and function prototypes
  
    
*/

#ifndef __BMP085_H__
#define __BMP085_H__

/*
	CHIP_TYPE CONSTANTS
*/

#define BMP085_CHIP_ID			0x55
#define BOSCH_PRESSURE_SMD500	05
#define BOSCH_PRESSURE_BMP085	85

/*
	BMP085 I2C Address
*/

#define BMP085_I2C_ADDR		(0xEE>>1)


/* 
 *	
 *	register definitions 	
 *
 */

#define BMP085_PROM_START__ADDR		0xaa
#define BMP085_PROM_DATA__LEN		  22

#define BMP085_CHIP_ID_REG			0xD0
#define BMP085_VERSION_REG			0xD1

#define BMP085_CTRL_MEAS_REG		0xF4
#define BMP085_ADC_OUT_MSB_REG		0xF6
#define BMP085_ADC_OUT_LSB_REG		0xF7

#define BMP085_SOFT_RESET_REG		0xE0

#define BMP085_T_MEASURE        0x2E				// temperature measurent 
#define BMP085_P_MEASURE        0x34				// pressure measurement

#define BMP085_TEMP_CONVERSION_TIME  5				// TO be spec'd by GL or SB


// BMP085 specific constants 

#define BMP085_PARAM_MG      3038        //calibration parameter
#define BMP085_PARAM_MH     -7357        //calibration parameter
#define BMP085_PARAM_MI      3791        //calibration parameter


/** this structure holds all device specific calibration parameters 
*/
typedef struct {
   short ac1;
   short ac2;
   short ac3;
   unsigned short ac4;
   unsigned short ac5;
   unsigned short ac6;
   short b1;
   short b2;
   short mb;
   short mc;
   short md;      		   
} bmp085_calibration_param_t;


/** BMP085 image registers data structure

*/
typedef struct  {	
	bmp085_calibration_param_t cal_param;	
	unsigned char mode;
	unsigned char chip_id,	ml_version,	al_version;

	long param_b5;
	int number_of_samples;
	short oversampling_setting;
	//short smd500_t_resolution, smd500_masterclock;

} bmp085_t;








	
/* 
 *	
 *	bit slice positions in registers
 *
 */

#define BMP085_CHIP_ID__POS		0
#define BMP085_CHIP_ID__MSK		0xFF
#define BMP085_CHIP_ID__LEN		8
#define BMP085_CHIP_ID__REG		BMP085_CHIP_ID_REG


#define BMP085_ML_VERSION__POS		0
#define BMP085_ML_VERSION__LEN		4
#define BMP085_ML_VERSION__MSK		0x0F
#define BMP085_ML_VERSION__REG		BMP085_VERSION_REG



#define BMP085_AL_VERSION__POS  	4
#define BMP085_AL_VERSION__LEN  	4
#define BMP085_AL_VERSION__MSK		0xF0
#define BMP085_AL_VERSION__REG		BMP085_VERSION_REG


/* DATA REGISTERS */



/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */



#define BMP085_GET_BITSLICE(regvar, bitname)\
			(regvar & bitname##__MSK) >> bitname##__POS


#define BMP085_SET_BITSLICE(regvar, bitname, val)\
		  (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)  


/* General Setup Functions */



/** BMP085_init 
   
   input : 	Pointer to bmp085_t 
   output:  -		
   return:  result of communication function
   notes :   
*/
int bmp085_init(vfpv_t delay1ms);

short bmp085_get_temperature(void);
unsigned long bmp085_read_up(void); // read raw value out of array
long bmp085_calc_pressure(unsigned long up);

// for inserting waits in between
uint32_t bmp085_TWI_trig_temp_meas(void); // return required wait time
void bmp085_TWI_trig_temp_read(void); // return nothing
uint32_t bmp085_TWI_trig_press_meas(void); // return wait time
void bmp085_TWI_trig_press_read(void); // return nothing

int32_t bmp085_calcHeight_mm(int32_t pBaroPas);





#endif   // __BMP085_H__





