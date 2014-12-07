/*
 * Hottv4.c
 *
 * extracted from a .pde: 25.08.2012 21:00:08
 * partially modified by Fabian Huslik
 */
//stuff taken from "ardupilot project" (LGPL)
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// HoTT v4 telemetry for ArduCopter
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//
// Check project homepage at https://code.google.com/p/hott-for-ardupilot/
//
// 08/2012 by
// Adam Majerczyk (majerczyk.adam@gmail.com)
//
// v0.9.3b (beta software)
//
// Developed and tested with:
// Transmitter MC-32 v1.030
// Receiver GR-16 v6a10_f9
// Receiver GR-12 v3a40_e9


#include <asf.h>
#include "hottv4.h"
//#include <stdio.h> // sprintf in there is POISON!!
#include <string.h>
#include "HoTTInterface.h"
#include "menu/menu.h"
#include "menu/menu_variant.h"
#include "servo_in.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/printf.h"
#include "version.h"
#include "TaskMenu.h"
#include "modules/GPS.h"
#include "modules/vector.h"
#include "TaskNavi.h"
#include "../TaskControl.h"

extern GPS_interface_t   gps_dataset; // fixme schönes Interface basteln

//HoTT protocol stuff taken from "ardupilot project" (LGPL)
// HoTT v4 telemetry for ArduCopter
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//
// Check project homepage at https://code.google.com/p/hott-for-ardupilot/
//
// 08/2012 by
// Adam Majerczyk (majerczyk.adam@gmail.com)
//
// v0.9.3b (beta software)
//


/************************************************************************/
/* Button stuff                                                         */
/************************************************************************/
static volatile int buttonValue;
static volatile int buttonValueMax;
static volatile int buttonValueMin;
static volatile int buttonValueStep;


// HoTT stuff
static uint8_t _hott_telemetry_is_sending = false;
//static uint8_t _hott_telemetry_sendig_msgs_id = 0;

//HoTT serial buffer

static volatile uint8_t _hott_serial_buffer[176]; // for tailing zeroes???

// HoTT serial send buffer pointer
static volatile uint8_t *_hott_msg_ptr = 0;
// Len of HoTT serial buffer
static volatile uint8_t _hott_msg_len = 0;

uint8_t menu_show_diag = 0; // todo clean

// prototypes only used within this file:
static void _hott_send_telemetry_data( void );
static void _hott_send_msg(volatile uint8_t *buffer ,volatile uint8_t len );
static void _hott_check_serial_data( uint32_t tnow );
static void _hott_enable_transmitter( void );
static void _hott_enable_receiver( void );
static void _hott_ser_flush( void );
static void _hott_ser_write( uint8_t );
static uint8_t _hott_ser_available( void );
static uint8_t _hott_ser_read( void );

#ifdef HOTT_SIM_TEXTMODE
static void _hott_send_text_msg(bool esc);
#endif
#ifdef HOTT_SIM_GAM_SENSOR
static void _hott_update_gam_msg(void);
#endif
#ifdef HOTT_SIM_GPS_SENSOR
static void _hott_update_gps_msg(void);
#endif

void _hott_msgs_init(void);
void convertToGraGL(int32_t deg, uint16_t* a, uint16_t* b);
uint8_t convert_gra_to_GraDeg(int16_t d);


//*****************************************************************************

void act_diag(void)
{
	menu_show_diag = 1;
}


static void _hott_enable_transmitter( void )
{
	HoTTEnableTransmitterMode();
}
static void _hott_enable_receiver( void )
{
	HoTTEnableReceiverMode();
}
static void _hott_ser_flush( void )
{
	HoTTFlushBuffer();
}
static void _hott_ser_write( uint8_t c )
{
	HoTTSendByte( c );
}
static uint8_t _hott_ser_available( void )
{
	return HoTTGetAvailable();
}
static uint8_t _hott_ser_read( void )
{
	return HoTTRecByte();
}

/*
 Onetime setup for HoTT
 */
void hott_init( void )
{
	// serial init with 19200 baud is already done.
	_hott_enable_receiver();
	//_hott_msgs_init();
}

/*
 Called periodically (˜1ms) by timer scheduler for RX and TX

 @param tnow  timestamp in uSecond
 */
void hott_serial_scheduler( uint32_t tnow )
{
	static uint32_t _hott_serial_timer;
	_hott_check_serial_data( tnow ); // Check for data request
	if(_hott_msg_ptr == 0) return; //no data to send
	if(_hott_telemetry_is_sending)
	{
		//we are sending already, wait for a delay of 2ms between data bytes
		if(tnow - _hott_serial_timer < 3000) //delay ca. 3,5 mS. 19200 baud = 520uS / Byte + 3ms required delay
		// Graupner Specs claims here 2ms but in reality it's 3ms, they using 3ms too...
		return;
	}
	else
	{
//new data request
		tnow = OS_GetTicks()*1000; //correct the 5ms  delay in _hott_check_serial_data()...
	}
	_hott_send_telemetry_data();
	_hott_serial_timer = tnow;
}

/*
 Transmitts a HoTT message
 */
static void _hott_send_telemetry_data( void )
{
	static uint8_t msg_crc = 0;
	if(!_hott_telemetry_is_sending)
	{
// new message to send
		_hott_telemetry_is_sending = true;
		_hott_enable_transmitter(); //switch to transmit mode  
	}

//send data
	if(_hott_msg_len == 0)
	{
//all data send
		_hott_msg_ptr = 0;
		_hott_telemetry_is_sending = false;
		msg_crc = 0;
		_hott_enable_receiver();
		_hott_ser_flush();
	}
	else
	{
		--_hott_msg_len;
		if(_hott_msg_len != 0)
		{
			msg_crc += *_hott_msg_ptr; 
			_hott_ser_write( *_hott_msg_ptr++ );
		}
		else
		{
//last byte, send crc
			_hott_ser_write( (uint8_t)msg_crc );
		}
	}
}


/*

//  Initializes a HoTT GPS message (Receiver answer type  !Not Smartbox)

void _hott_msgs_init(void) {
#ifdef HOTT_SIM_GPS_SENSOR
  memset(hott_gps_msg, 0, sizeof(HOTT_GPS_MSG_t));
  hott_gps_msg->start_byte = 0x7c;
  hott_gps_msg->gps_sensor_id = 0x8a;
  hott_gps_msg->sensor_id = 0xa0;
 
  hott_gps_msg->version = 0x00;
  hott_gps_msg->end_byte = 0x7d;
#endif

#ifdef HOTT_SIM_EAM_SENSOR
 memset(&hott_eam_msg, 0, sizeof(HOTT_EAM_MSG_t));
 hott_eam_msg.start_byte = 0x7c;
 hott_eam_msg.eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
 hott_eam_msg.sensor_id = 0xe0;
 hott_eam_msg.stop_byte = 0x7d;
#endif

#ifdef HOTT_SIM_VARIO_SENSOR
 memset(&hott_vario_msg, 0, sizeof(HOTT_VARIO_MSG_t));
 hott_vario_msg.start_byte = 0x7c;
 hott_vario_msg.vario_sensor_id = HOTT_TELEMETRY_VARIO_SENSOR_ID;
 hott_vario_msg.sensor_id = 0x90;
 hott_vario_msg.stop_byte = 0x7d;
#endif

#ifdef HOTT_SIM_GAM_SENSOR
 memset(&hott_gam_msg, 0, sizeof(HOTT_GAM_MSG_t));
 hott_gam_msg.start_byte = 0x7c;
 hott_gam_msg.gam_sensor_id = HOTT_TELEMETRY_GAM_SENSOR_ID;
 hott_gam_msg.sensor_id = 0xd0;
 hott_gam_msg.stop_byte = 0x7d;
#endif
 
#ifdef HOTT_SIM_TEXTMODE
  memset(&hott_txt_msg, 0, sizeof(HOTT_TEXTMODE_MSG_t));
  hott_txt_msg.start_byte = 0x7b;
  hott_txt_msg.fill1 = 0x00;
  hott_txt_msg.warning_beeps = 0x00;
  hott_txt_msg.stop_byte = 0x7d;
#endif
}
*/



/*
 Check for a valid HoTT requests on serial bus
 */
static void _hott_check_serial_data( uint32_t tnow )
{
	static uint32_t _hott_serial_request_timer = 0;
	if(_hott_telemetry_is_sending == true) return;
	if(_hott_ser_available() > 1)
	{
		if(_hott_ser_available() == 2)
		{
			if(_hott_serial_request_timer == 0)
			{
				//new request, check required
				_hott_serial_request_timer = tnow; //save timestamp
				return;
			}
			else
			{
				if(tnow - _hott_serial_request_timer < 4600) //wait ca. 5ms
				{
					return;
				}					
				_hott_serial_request_timer = 0; //clean timer
			}
			// we never reach this point if there is additionally data on bus, so is has to be valid request
			unsigned char c = _hott_ser_read();
			unsigned char addr = _hott_ser_read();
		
			//ok, we have a valid request, check for address
			switch(c)
			{
//*****************************************************************************
#ifdef HOTT_SIM_TEXTMODE
				case HOTT_TEXT_MODE_REQUEST_ID:
//Text mode
				{
					if(addr >> 4 != (HOTT_TELEMETRY_GPS_SENSOR_ID & 0x0f))
					{
						_hott_ser_flush();
						_hott_serial_request_timer = 0;
						break; // only answer GPS text mode
					}
					
					
					HOTT_TEXTMODE_MSG_t *hott_txt_msg = (HOTT_TEXTMODE_MSG_t *)&_hott_serial_buffer[0];
					//memset(hott_txt_msg, 0, sizeof(HOTT_TEXTMODE_MSG_t));
					TMNU_SetPtr(hott_txt_msg->msg_txt);
				
					bool b_esc= false;
				
					// key handling
					switch (addr&0x0f) // low nibble
					{
						case HOTT_TEXT_MODE_IDLE: 
							// do nothing here, as this means that no button is pressed.
							TMNU_TriggerMenu(0,0);
							break;
						case HOTT_TEXT_MODE_RIGHT:
							TMNU_TriggerMenu('P',0);
							//button_FastPlus();
							break;
						case HOTT_TEXT_MODE_UP: 
							TMNU_TriggerMenu('p',0);
							//button_Plus();
							break;
						case HOTT_TEXT_MODE_LEFT:
							//button_FastMinus();
							TMNU_TriggerMenu('M',0);
							break;
						case HOTT_TEXT_MODE_DOWN: 
							//button_Minus();
							TMNU_TriggerMenu('m',0);
							break;
						case HOTT_TEXT_MODE_SET:
							if(menu_show_diag == 1)
							{
								menu_show_diag = 0;
							}
							else
							{
								//menu_select();
								TMNU_TriggerMenu('E',0);		
							}
							break;
						case HOTT_TEXT_MODE_ESC:
							b_esc = true;
							break;
						default:
							break;
					}
					
					
			
					/************************************************************************/
					/* The menu interface                                                   */
					/************************************************************************/
						
					// menue is filled in Task menue
						
										


					_hott_send_text_msg(b_esc);//send message
				}
#endif
				break;
//*****************************************************************************
			case HOTT_BINARY_MODE_REQUEST_ID:
				TMNU_SetPtr(NULL);
#ifdef HOTT_SIM_GPS_SENSOR
//GPS module binary mode
				if(addr == HOTT_TELEMETRY_GPS_SENSOR_ID)
				{
//            Serial.printf("\n%ld: REQ_GPS",micros() / 1000);
					_hott_update_gps_msg();
					_hott_send_msg(_hott_serial_buffer, sizeof(HOTT_GPS_MSG_t));
					break;
				}
#endif
#ifdef HOTT_SIM_EAM_SENSOR
				if(addr == HOTT_TELEMETRY_EAM_SENSOR_ID)
				{
//            Serial.printf("\n%ld: REQ_EAM",micros() / 1000);
					_hott_update_eam_msg();
					_hott_send_msg(_hott_serial_buffer, sizeof(HOTT_EAM_MSG_t));
					break;
				}
#endif
#ifdef HOTT_SIM_VARIO_SENSOR
				if(addr == HOTT_TELEMETRY_VARIO_SENSOR_ID)
				{
//            Serial.printf("\n%ld: REQ_VARIO",micros() / 1000);
					_hott_update_vario_msg();
					_hott_send_msg(_hott_serial_buffer, sizeof(HOTT_VARIO_MSG_t));
					break;
				}
#endif
#ifdef HOTT_SIM_GAM_SENSOR
				if(addr == HOTT_TELEMETRY_GAM_SENSOR_ID)
				{
//            Serial.printf("\n%ld: REQ_GAM",micros() / 1000);
					_hott_update_gam_msg();
					_hott_send_msg(_hott_serial_buffer, sizeof(HOTT_GAM_MSG_t));
					break;
				}
#endif

//END: case HOTT_BINARY_MODE_REQUEST_ID:
				break;
//*****************************************************************************         
			default:
//Serial.printf("0x%02x Mode for 0x%02x\n", c, addr);
				_hott_ser_flush();
				TMNU_SetPtr(NULL);
				_hott_serial_request_timer = 0;
				break;
			}
		}
		else // == 2
		{
//ignore data from other sensors
			_hott_ser_flush();
			_hott_serial_request_timer = 0;
		}
	
	} // > 1
}

static void _hott_send_msg(volatile uint8_t *buffer ,volatile uint8_t len ) // fixme "buffer" unused???
{
	if(_hott_telemetry_is_sending == true) return;
	_hott_msg_ptr = buffer;
	_hott_msg_len = len + 1; //len + 1 byte for crc
}

#ifdef HOTT_SIM_TEXTMODE
static void _hott_send_text_msg(bool esc) // fixme esc unused
{
	HOTT_TEXTMODE_MSG_t* hott_txt_msg = (HOTT_TEXTMODE_MSG_t*)&_hott_serial_buffer[0];
	// clean up the string before sending. otherwise the Graupner transmitter displays garbage in '0' spaces.
	for (int i =0; i< HOTT_TEXTMODE_MSG_TEXT_LEN-1;i++)
	{
		if (hott_txt_msg->msg_txt[i] == 0)
		{
			hott_txt_msg->msg_txt[i] = ' ';
		}
		if (hott_txt_msg->msg_txt[i] == 0x80 )
		{
			hott_txt_msg->msg_txt[i] = ' ' | 0x80;
		}
	}
	
	hott_txt_msg->start_byte = 0x7b;
	hott_txt_msg->warning_beeps = 0x0; // otherwise it beeps randomly
	hott_txt_msg->fill1 = 0xD0; //esc?0xef:0xe0; // use 0x0e for normal, 0xef for esc.
	hott_txt_msg->stop_byte = 0x7d;
	
	
	_hott_send_msg(_hott_serial_buffer, sizeof(HOTT_TEXTMODE_MSG_t));
	//_hott_telemetry_sendig_msgs_id = HOTT_TEXT_MODE_REQUEST_ID;
}
#endif

/*
  Converts a "black on white" string into inverted one "white on black"
  Works in text mode only!
*/
char * hott_invert_chars(char *str, int cnt) {
	//if(str == 0) return str;
	//int len = strlen(str);
	//if((len < cnt)  && cnt > 0) len = cnt;
	//for(int i=0; i< len; i++) {
	for(int i=0; i< cnt; i++) {
		str[i] = (uint8_t)(0x80 + (uint8_t)str[i]);
	}
	return str;
}

/*char * hott_invert_all_chars( char *str )
{
	return hott_invert_chars( str, 0 );
}*/

#ifdef HOTT_SIM_GAM_SENSOR
#ifdef HOTT_ALARMS
static byte _hott_gam_voice_alarm = 0;
static byte _hott_gam_alarm_invers1 = 0;
static byte _hott_gam_alarm_invers2 = 0;
#endif

static void _hott_update_gam_msg(void)
{
	uint16_t temp;
	HOTT_GAM_MSG_t *hott_gam_msg = (HOTT_GAM_MSG_t *)&_hott_serial_buffer[0];
	memset(hott_gam_msg, 0, sizeof(HOTT_GAM_MSG_t));
	hott_gam_msg->start_byte = 0x7c;
	hott_gam_msg->gam_sensor_id = HOTT_TELEMETRY_GAM_SENSOR_ID;
	hott_gam_msg->sensor_id = 0xd0;
	hott_gam_msg->stop_byte = 0x7d;

	hott_gam_msg->cell1 = 210; // 210 = 4200mV -> 0.02V steps
	hott_gam_msg->cell2 = 210;
	hott_gam_msg->cell3 = 200;
	hott_gam_msg->cell4 = 210;
	hott_gam_msg->cell5 = 210;
	hott_gam_msg->cell6 = 210;

#ifdef HOTT_ALARMS
	hott_gam_msg->warning_beeps = _hott_gam_voice_alarm;
	hott_gam_msg->alarm_invers1 = _hott_gam_alarm_invers1;
	hott_gam_msg->alarm_invers2 = _hott_gam_alarm_invers2;
#endif
	hott_gam_msg->temperature1 = 20; // 20 = 0°C
	hott_gam_msg->temperature2 = 20; // 20 = 0°C
	temp = 500; // 500 = 0
	hott_gam_msg->altitude_L = temp & 0x00ff;
	hott_gam_msg->altitude_H = (temp & 0xff00)>>8;
	temp = 30000 + 0;// = actual CR = 0
	hott_gam_msg->climbrate_L = temp & 0x00ff;
	hott_gam_msg->climbrate_H = (temp & 0xff00)>>8;
	hott_gam_msg->climbrate3s = 120 + (0 / 100);// 0 m/3s using filtered data here
	temp = 10;
	hott_gam_msg->current_L = temp & 0x00ff;
	hott_gam_msg->current_H = (temp & 0xff00)>>8;
	temp = 100;
	hott_gam_msg->main_voltage_L = temp & 0x00ff;
	hott_gam_msg->main_voltage_H = (temp & 0xff00)>>8;
	temp = 10;
	hott_gam_msg->batt_cap_L = temp & 0x00ff;
	hott_gam_msg->batt_cap_H = (temp & 0xff00)>>8;
	hott_gam_msg->fuel_procent = 60;// my fuel are electrons :)
	temp = (int)((float)(7.0 * 0.036)); // 7 = km/h
	hott_gam_msg->speed_L = temp & 0x00ff;
	hott_gam_msg->speed_H = (temp & 0xff00)>>8;

//display ON when motors are armed
	if (1)
	{
		hott_gam_msg->alarm_invers2 |= 0x80;
	}
	else
	{
		hott_gam_msg->alarm_invers2 &= 0x7f;
	}
}
#endif  //HOTT_SIM_GAM_SENSOR


// 10000000
// 48.38281600
// 
// 4838281600
// N 48°22.969'
// Naaa°aa.bbbb
void convertToGraGL(int32_t deg, uint16_t* a, uint16_t* b)
{
	int32_t t;
	int32_t t2;
	uint32_t res_a,res_b;

	t = deg;
	res_a = deg / 10000000; // this are full degrees;
	t -= res_a * 10000000; // this is the rest of the value minus the full degrees.
	res_a *= 100; // soo, we have the full degrees at the right pos.
	
	// now the minutes:
	t *= 60; // now this is the rest in minutes
	t2 = t / 10000000; // full min
	res_a += t2;
	t -= t2*10000000;
	res_b = t / 1000;
	
	*a = res_a;
	*b = res_b;

}

uint8_t convert_gra_to_GraDeg(int16_t deg)
{
	uint8_t ret;
	
	// unwind
	while(deg > 360)
		deg -= 360;
	
	while(deg < 0)
		deg += 360;
	
	ret = deg/2;

	return ret;
}


#ifdef HOTT_SIM_GPS_SENSOR
// Updates GPS message values
void _hott_update_gps_msg() {
	
	uint16_t GraGLaaaa = 0;
	uint16_t GraGLbbbb = 0;
	
	HOTT_GPS_MSG_t *hott_gps_msg = (HOTT_GPS_MSG_t *)&_hott_serial_buffer[0];
	memset(hott_gps_msg, 0, sizeof(HOTT_GPS_MSG_t));
	
	hott_gps_msg->start_byte = 0x7c;
	hott_gps_msg->gps_sensor_id = 0x8a;
	hott_gps_msg->sensor_id = 0xa0;
 
	hott_gps_msg->version = 0x0;
	hott_gps_msg->end_byte = 0x7d;
	
	
	
	// update GPS telemetry data
	hott_gps_msg->msl_altitude_H = ((int32_t)gps_dataset.altitude + 500) >> 8;  //meters above sea level -500	
	hott_gps_msg->msl_altitude_L = ((int32_t)gps_dataset.altitude + 500) & 0xff;  

	uint16_t gpsspeed = gps_dataset.ground_speed_mps/3.6;
	hott_gps_msg->gps_speed_H = gpsspeed >> 8;
	hott_gps_msg->gps_speed_L = gpsspeed & 0xff;

	if(gps_dataset.status.gps2dfix) 
	{
		hott_gps_msg->alarm_invers2 = 0;
		hott_gps_msg->gps_fix_char = 'x';
		hott_gps_msg->free_char3 = gps_dataset.status.gps3dfix?'3':'2';  //3D Fix according to specs...
		
// fix		uint16_t homedist = NAV_info.Dist_m;
				
// fix		hott_gps_msg->home_distance_H = homedist >> 8;
// fix		hott_gps_msg->home_distance_L = homedist  & 0xff;
		
		int16_t homedir;
// fix		homedir = NAV_info.trg_heading_rad * 57.295779513;
		homedir = 360 - homedir;
		homedir /=2;
		hott_gps_msg->home_direction = homedir; // 2° steps
		
	} 
	else 
	{
		//No GPS Fix
		hott_gps_msg->alarm_invers2 = 1;
		hott_gps_msg->gps_fix_char = '-';
		hott_gps_msg->free_char3 = '-';
		hott_gps_msg->home_distance_H = 0; // set distance to 0 since there is no GPS signal
		hott_gps_msg->home_distance_L = 0;
		hott_gps_msg->home_direction = 0; // 2° steps
	}
	

	//home position
	hott_gps_msg->free_char1 =' '; // could also be N E S W (near "m")
	hott_gps_msg->free_char2 =' '; // could also be N E S W (near "°")
	
	
	
	if(gps_dataset.gps_loc.lat>0) {
		hott_gps_msg->pos_NS = 0;  //north
		} else {
		hott_gps_msg->pos_NS = 1;  //south
	}
	
	convertToGraGL (gps_dataset.gps_loc.lat,&GraGLaaaa,&GraGLbbbb);
	
	hott_gps_msg->pos_NS_aa_H = GraGLaaaa >> 8;  //lat;	
	hott_gps_msg->pos_NS_aa_L = GraGLaaaa & 0xff; //lat;
	hott_gps_msg->pos_NS_bb_H = GraGLbbbb >> 8; //lat_sec;
	hott_gps_msg->pos_NS_bb_L = GraGLbbbb & 0xff;
	
	if(gps_dataset.gps_loc.lon>0) {
		hott_gps_msg->pos_EW = 0; //east
		} else {
		hott_gps_msg->pos_EW = 1; //west
	}

	convertToGraGL (gps_dataset.gps_loc.lon, &GraGLaaaa,&GraGLbbbb);
	
	hott_gps_msg->pos_EW_aa_H = GraGLaaaa >> 8; // lon
	hott_gps_msg->pos_EW_aa_L = GraGLaaaa & 0xff;
	hott_gps_msg->pos_EW_bb_H = GraGLbbbb >> 8;
	hott_gps_msg->pos_EW_bb_L = GraGLbbbb & 0xff;
	
	hott_gps_msg->altitude_H = (IMUdata.height_dm/10 + 500) >> 8;  //meters above ground +500
	hott_gps_msg->altitude_L = (IMUdata.height_dm/10 + 500) & 0xff;  //meters above ground +500

	hott_gps_msg->climbrate_H = 30000 >> 8; // climb_rate +30000 ;
	hott_gps_msg->climbrate_L = 30000  & 0xff; // climb_rate +30000 ;
	hott_gps_msg->climbrate3s = 120;  // climb +120 m/3s
	
	hott_gps_msg->gps_satelites = GPS_GetNumSats();
	
	hott_gps_msg->angle_roll = convert_gra_to_GraDeg(IMUdata.roll_deg);
	hott_gps_msg->angle_nick = convert_gra_to_GraDeg(IMUdata.pitch_deg);
	
	hott_gps_msg->angle_compass = convert_gra_to_GraDeg(360-IMUdata.mag_heading_deg); // 2° steps // does not aapear somewhere
	hott_gps_msg->flight_direction = convert_gra_to_GraDeg(360-IMUdata.mag_heading_deg); // 2° steps // "Dir:"

	uint32_t time = gps_dataset.time;
	hott_gps_msg->gps_time_h = time/10000000;
	time %=10000000;
	hott_gps_msg->gps_time_m = time/100000;
	time %=100000;
	hott_gps_msg->gps_time_s = time/1000;
	time %=1000;
	hott_gps_msg->gps_time_sss = time;
}
#endif



/************************************************************************/
/* Button stuff                                                         */
/************************************************************************/
uint16_t GetvalueFromUI(void)
{
	return buttonValue;
}

void HandOverValueToUI(int16_t value, int16_t upper, int16_t lower, int16_t stepsize)
{
	buttonValue = value;
	buttonValueMax = upper;
	buttonValueMin = lower;
	buttonValueStep = stepsize;
}



void button_FastPlus()
{
	if(buttonValue+buttonValueStep*5 <= buttonValueMax)
	buttonValue += buttonValueStep*5;
	else
	buttonValue = buttonValueMax;
}

void button_Plus()
{
	if(buttonValue<buttonValueMax)buttonValue+=buttonValueStep;
}

void button_FastMinus()
{
	if(buttonValue-buttonValueStep*5 >= buttonValueMin)
	buttonValue-=buttonValueStep*5;
	else
	buttonValue = buttonValueMin;
}

void button_Minus()
{
	if(buttonValue>buttonValueMin)buttonValue-=buttonValueStep;
}