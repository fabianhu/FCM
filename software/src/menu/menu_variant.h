/* 
 * Definitions for FCM menu 
 * 
 * (c) 2012-2014 Fabian Huslik
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
 * 
 * 
 * WARNING: Parts of this file are auto-generated with Treebuilder.exe, 
 * do not edit the generated section manually, as you WILL loose all changes.
 */


#define MENU_DRAWALWAYS 1

#include <string.h>
#include "version.h"

// kill AVR 8 bit-stuff 
#define strcpy_P strcpy
#define PROGMEM

// Potmeter setting 
#define POTDIVISOR40 250 // gives 0..40 (40 steps in MX-20 poti)
#define POTDIVISOR100 100 // 0..100
#define POTDIVISOR200 50 // 0..200

//Menusettings
#define MENU_ITEMS_PER_PAGE 	6	//nr of items per menu page (without header)


//get menu item name location
#define FLASH	0
#define SRAM	1
#define EEPROM	2

//extern Prototypes
extern void menu_draw_header(char *menu_header);
extern void menu_del_menuitems(void);
extern void menu_draw_unselected_items(char *item_name, Parameter_t* par, uint8_t lcd_pos);
extern void menu_draw_selected_item(char *item_name, Parameter_t* par, uint8_t lcd_pos);
extern void menu_draw_selected_parameter(char *item_name, Parameter_t* par, uint8_t lcd_pos);
extern void menu_draw_groupposition(uint8_t itemnr, uint8_t groupitems);

typedef struct Parinfo_tag // this is more for FCMCP than for menue!
{
	uint8_t ID;
	uint8_t Parent;
	int16_t value;
	int16_t upper;
	int16_t lower;
	char text[20];//MAX_ITEM_NAME_CHARLENGTH+1];
}Parinfo_t;

void menu_setPtr(char* ptr);

extern volatile uint8_t g_CalibrationRequest;
void UpdatePotsFromTX(void);
void GetPotiTxt( char* ptxt );
void GetDiagTxt( char* ptxt );
bool PAR_SendNextItem (Parinfo_t* pi);
void menue_setParVal(uint8_t id,int16_t val);
void menue_sendpar(uint8_t item);




//******** START OF AUTO-GENERATED HEADER DO NOT EDIT!!! *********
//********          Generated with Treebuilder.exe       *********

 #define	MENUESIZE	117	// number of menu itmes (array size)
 #define	MAX_ITEM_NAME_CHARLENGTH	17	// max name length
 #define	MENUGENUID	42576	// Generation UID
// Enum definitions
typedef enum
{
	ch,
	m,
	mps,
	none,
	perc,
	s,
} eParameterType_t;

// Parameter externals
typedef struct myPar_tag
{
      Parameter_t integrator_limit;	// Max Integrator windup (4000)
      Parameter_t govAtMax;	// Governor intensity at max power [%] (20)
      Parameter_t quat_amp;	// Quaternion turn rate (4000)
      Parameter_t pid_r_p;	// (60)
      Parameter_t pid_r_i;	// (37)
      Parameter_t pid_r_d;	// (37)
      Parameter_t pid_y_p;	// (58)
      Parameter_t pid_y_i;	// (21)
      Parameter_t pid_y_d;	// (16)
      Parameter_t nav_max_angle;	// The max lean angle in degrees (30)
      Parameter_t nav_max_acc_lim;	// The max acceleration out of nav in 110 ms^2  (2)
      Parameter_t nav_max_accelint;	// The max acceleration integrator windup out of nav in 110  ms^2  (2)
      Parameter_t nav_decel_radius;	// The radius in meters, at which the cruise speed starts to decelerate (20)
      Parameter_t nav_decel_radiush;	// The radius in meters, at which the height change speed starts to decelerate (20)
      Parameter_t nav_acc_flt_glob;	// global acceleration filter value (1000) 1000 is unfiltered
      Parameter_t nav_alpha_Pos;	// complementary filter for position (9990) 10000 is accel prio
      Parameter_t pid_nav_mislead_on;	// Activate the compensation algorithm
      Parameter_t pid_nav_mislead_minspeed;	// Minimum speed, at which the vectoring calculation starts (1)
      Parameter_t pid_nav_mislead_mintime;	// Minimum time en route, after which the vectoring calculation starts (5)
      Parameter_t pid_nav_mislead_filter;	// 100 is unfiltered
      Parameter_t pid_nav_mislead_mindist;	// Minimum distance from target to activate the compensation
      Parameter_t pid_nav_p;	// (25)
      Parameter_t pid_nav_i;	// (0)
      Parameter_t pid_nav_d;	// (30)
      Parameter_t pid_h_p;	// (300)
      Parameter_t pid_h_i;	// (50)
      Parameter_t pid_h_d;	// (1000)
      Parameter_t max_power;	// Max throttle (6500)
      Parameter_t idle_power;	// Min throttle, use above 1500 for idle running props
      Parameter_t Agilityfactor;	// How aggressive the copter reacts to RC (1-2)
      Parameter_t waitGPSFix;	// 1 = Flight is only released if GPS is fixed 
      Parameter_t NoConfigBTM222atStart;	// 1 = skip BTM222 initialisation sequence at start
      Parameter_t DirectMode;	// 1 = NO Flight control, throttle directly & without unlock to all ESC! Use this to calibrate all ESCs at once. Negative numbers = single motor control, others off
      Parameter_t dsmx_on;	// 1= try bind in DSMX mode
      Parameter_t cal_gyro_filter;	// Gyroscope filter: 1= unfiltered (1)
      Parameter_t cal_acc_filter;	// Accelerometer filter: 1= unfiltered (4)
      Parameter_t cal_mag_filter;	// Magnetometer filter: 1= unfiltered   (4)
      Parameter_t cal_baro_filter;	// Barometer filter: 1000= unfiltered   (990)
      Parameter_t cal_gps_filter;	// GPS filter: 1000= unfiltered  (990)
      Parameter_t cal_gyro_x;	// Gyro calibration value
      Parameter_t cal_gyro_y;	// Gyro calibration value
      Parameter_t cal_gyro_z;	// Gyro calibration value
      Parameter_t cal_acc_x;	// Accelometer calibration value
      Parameter_t cal_acc_y;	// Accelometer calibration value
      Parameter_t cal_acc_z;	// Accelometer calibration value
      Parameter_t cal_mag_auto_inflight;	// Magnetometer calibration during engines on allowed ? (0)
      Parameter_t cal_mag_x;	// Magnetometer calibration value (via online calibration)
      Parameter_t cal_mag_y;	// Magnetometer calibration value (via online calibration)
      Parameter_t cal_mag_z;	// Magnetometer calibration value (via online calibration)
      Parameter_t SwitchPosHold;	// Which channel is used for Pos hold (ch #)
      Parameter_t SwPPosHold;	// At which pos is the function active (0=lo 1=mid 2=hi) Pos Hold wins over RTH!
      Parameter_t SwitchHGov;	// Which channel is used for height governor on (ch #)
      Parameter_t SwPHGov;	// At which pos is the function active (0=lo 1=mid 2=hi)
      Parameter_t SwitchReturnToHome;	// Which channel is used for Return To Home (ch #)
      Parameter_t SwPReturnToHome;	// At which pos is the function active (0=lo 1=mid 2=hi)
      Parameter_t SwitchAux;	// Which channel is used for Auxiliary function (ch #)
      Parameter_t SwPAux;	// At which pos is the function active (0=lo 1=mid 2=hi)
      Parameter_t PotiP;	// Which channel is used for P potmeter
      Parameter_t PotiI;	// Which channel is used for I potmeter
      Parameter_t PotiD;	// Which channel is used for D potmeter
      Parameter_t madgwick;	// 
      Parameter_t a_madg;	// x 0.01 --> 100 = 1.0 (10)
      Parameter_t P_mahony;	// x 0.01 --> 100 = 1.0 (50)
      Parameter_t I_mahony;	// x 0.01 --> 100 = 1.0 (10)
      Parameter_t test_P;	// 
      Parameter_t test_I;	// 
      Parameter_t test_D;	// 
      Parameter_t wind_freq;	// 
      Parameter_t wind_ampl;	// 
      Parameter_t mag_mis_freq;	// 
      Parameter_t mag_mis_ampl;	// 
      Parameter_t gyro_freq;	// 
      Parameter_t gyro_ampl;	// 
      Parameter_t accel_freq;	// 
      Parameter_t accel_ampl;	// 
      Parameter_t magneto_freq;	// 
      Parameter_t magneto_ampl;	// 
} myPar_t;

extern myPar_t myPar;

 #define	MENUE_PARCOUNT	77	// number of parameters

// Action Prototypes
void actSwashasPot (void);	// Use Potis as PID settings for Swash (x&y)
void actYawasPot (void);	// Use Potis as PID settings for Yaw (z)
void actNAVasPot (void);	// Use Potis as PID settings
void actHasPot (void);	// Use potis as PID settings for height governor
void actBindSPEKTRUM (void);	// Do bind Spectrum Sattelite at RC-Port
void actStartBootloader (void);	// Go into bootloader mode
void actCalibrate (void);	// Calibrate Gyro and Accelometer ? put on flat surface!
void actSaveToFlash (void);	// Saves all settings & values to flash
void act_diag (void);	// Display diag page (press enter to leave)

// Text definitions
#ifndef MENUE_TEXT_VARDEF
#define MENUE_TEXT_VARDEF \
char	txtFCMSETTINGS[] 	PROGMEM="FCM 2.1 Settings"; \
char	txtGYROPID[] 	PROGMEM="GYRO PID"; \
char	txtNAV[] 	PROGMEM="NAV"; \
char	txtSETTINGS[] 	PROGMEM="Settings"; \
char	txtCALIBRATION[] 	PROGMEM="Calibration"; \
char	txtRCCHANNELS[] 	PROGMEM="RC Channels"; \
char	txtSAVETOFLASH[] 	PROGMEM="*Save to Flash"; \
char	txtDIAGMODE[] 	PROGMEM="*Diag mode"; \
char	txtTEST[] 	PROGMEM="Test"; \
char	txtBACK[] 	PROGMEM="<back>"; \
char	txtMAXINTEGR[] 	PROGMEM="Max Integr"; \
char	txtGOVATMAX[] 	PROGMEM="Gov at Max"; \
char	txtQGAIN[] 	PROGMEM="Q Gain"; \
char	txtSWASHASPOTI[] 	PROGMEM="*Swash as Poti"; \
char	txtSWASHP[] 	PROGMEM="Swash P"; \
char	txtSWASHI[] 	PROGMEM="Swash I"; \
char	txtSWASHD[] 	PROGMEM="Swash D"; \
char	txtYAWASPOTI[] 	PROGMEM="*Yaw as Poti"; \
char	txtYAWP[] 	PROGMEM="Yaw P  "; \
char	txtYAWI[] 	PROGMEM="Yaw I  "; \
char	txtYAWD[] 	PROGMEM="Yaw D  "; \
char	txtNAVLIMITS[] 	PROGMEM="NAV Limits"; \
char	txtNAVFILTERS[] 	PROGMEM="NAV Filters"; \
char	txtNAVCOMPENSATION[] 	PROGMEM="NAV Compensation"; \
char	txtNAVASPOTI[] 	PROGMEM="*NAV as Poti"; \
char	txtNAVP[] 	PROGMEM="NAV P"; \
char	txtNAVI[] 	PROGMEM="NAV I"; \
char	txtNAVD[] 	PROGMEM="NAV D"; \
char	txtHASPOTI[] 	PROGMEM="*H as Poti"; \
char	txtHP[] 	PROGMEM="H P  "; \
char	txtHI[] 	PROGMEM="H I  "; \
char	txtHD[] 	PROGMEM="H D  "; \
char	txtMAXANGLE[] 	PROGMEM="Max Angle"; \
char	txtMAXACCEL[] 	PROGMEM="Max Accel"; \
char	txtMAXACCINT[] 	PROGMEM="Max Acc int"; \
char	txtRADIUSNAV[] 	PROGMEM="Radius NAV"; \
char	txtRADIUSH[] 	PROGMEM="Radius h"; \
char	txtACCFLTGLOB[] 	PROGMEM="Acc Flt glob"; \
char	txtALPHAPOS[] 	PROGMEM="alpha pos"; \
char	txtCOMPON[] 	PROGMEM="Comp On"; \
char	txtMINSPEED[] 	PROGMEM="Min Speed"; \
char	txtMINTIME[] 	PROGMEM="Min Time"; \
char	txtFILTER[] 	PROGMEM="Filter"; \
char	txtMINDISTANCE[] 	PROGMEM="Min Distance"; \
char	txtMAXPOWER[] 	PROGMEM="Max Power  "; \
char	txtIDLEPOWER[] 	PROGMEM="Idle Power "; \
char	txtAGILITYFACT[] 	PROGMEM="Agility fact"; \
char	txtWAITGPSFIX[] 	PROGMEM="wait GPS Fix"; \
char	txtNOBTCFG[] 	PROGMEM="No BT222 cfg"; \
char	txtDIRECTMODE[] 	PROGMEM="Direct Mode"; \
char	txtSPEKTRUMDSMX[] 	PROGMEM="SPEKTRUM DSMX"; \
char	txtBNDSPEKTRUM[] 	PROGMEM="*Bnd SPEKTRUM"; \
char	txtSTARTBOOTLOADER[] 	PROGMEM="*Start Bootloader"; \
char	txtFILTERS[] 	PROGMEM="Filters"; \
char	txtCALIBRATERA[] 	PROGMEM="*Calibrate R+A"; \
char	txtGYROX[] 	PROGMEM="Gyro X"; \
char	txtGYROY[] 	PROGMEM="Gyro Y"; \
char	txtGYROZ[] 	PROGMEM="Gyro Z"; \
char	txtACCX[] 	PROGMEM="Acc X"; \
char	txtACCY[] 	PROGMEM="Acc Y"; \
char	txtACCZ[] 	PROGMEM="Acc Z"; \
char	txtMAGCALFLGHT[] 	PROGMEM="MagCal flght"; \
char	txtMAGX[] 	PROGMEM="Mag X"; \
char	txtMAGY[] 	PROGMEM="Mag Y"; \
char	txtMAGZ[] 	PROGMEM="Mag Z"; \
char	txtGYROFILTER[] 	PROGMEM="Gyro Filter"; \
char	txtACCELFILTER[] 	PROGMEM="Accel Filter"; \
char	txtMAGNETOFLT[] 	PROGMEM="Magneto Flt"; \
char	txtBAROFILTER[] 	PROGMEM="Baro Filter"; \
char	txtGPSFILTER[] 	PROGMEM="GPS Filter"; \
char	txtSWPOSHOLD[] 	PROGMEM="Sw  PosHold"; \
char	txtSWPPOSHOLD[] 	PROGMEM="SwP PosHold"; \
char	txtSWHGOV[] 	PROGMEM="Sw  H Gov"; \
char	txtSWPHGOV[] 	PROGMEM="SwP H Gov"; \
char	txtSWRTH[] 	PROGMEM="Sw  RTH"; \
char	txtSWPRTH[] 	PROGMEM="SwP RTH"; \
char	txtSWAUX[] 	PROGMEM="Sw  Aux"; \
char	txtSWPAUX[] 	PROGMEM="SwP Aux"; \
char	txtPOTIP[] 	PROGMEM="Poti P"; \
char	txtPOTII[] 	PROGMEM="Poti I"; \
char	txtPOTID[] 	PROGMEM="Poti D"; \
char	txtMADGWICK[] 	PROGMEM="Madgwick"; \
char	txtALPHAMADG[] 	PROGMEM="alpha Madg"; \
char	txtPROPMAH[] 	PROGMEM="prop Mah"; \
char	txtINTEGRMAH[] 	PROGMEM="integr Mah"; \
char	txtTESTP[] 	PROGMEM="test P"; \
char	txtTESTI[] 	PROGMEM="test I"; \
char	txtTESTD[] 	PROGMEM="test D"; \
char	txtWIND[] 	PROGMEM="wind"; \
char	txtMAGMIS[] 	PROGMEM="mag_mis"; \
char	txtGYRO[] 	PROGMEM="gyro"; \
char	txtACCEL[] 	PROGMEM="accel"; \
char	txtMAGNETO[] 	PROGMEM="magneto"; \
char	txtWINDFREQ[] 	PROGMEM="wind_freq"; \
char	txtWINDAMPL[] 	PROGMEM="wind_ampl"; \
char	txtMAGMISFREQ[] 	PROGMEM="mag_mis_freq"; \
char	txtMAGMISAMPL[] 	PROGMEM="mag_mis_ampl"; \
char	txtGYROFREQ[] 	PROGMEM="gyro_freq"; \
char	txtGYROAMPL[] 	PROGMEM="gyro_ampl"; \
char	txtACCELFREQ[] 	PROGMEM="accel_freq"; \
char	txtACCELAMPL[] 	PROGMEM="accel_ampl"; \
char	txtMAGNETOFREQ[] 	PROGMEM="magneto_freq"; \
char	txtMAGNETOAMPL[] 	PROGMEM="magneto_ampl"; \

#endif

// Parameter definitions
#ifndef MENUE_PARAM_VARDEF
#define MENUE_PARAM_VARDEF \
myPar_t myPar = { \
/*integrator_limit*/ {	0, 0, 10000, 100, perc}, \
/*govAtMax*/ {	0, 0, 100, 1, perc}, \
/*quat_amp*/ {	0, 0, 10000, 100, perc}, \
/*pid_r_p*/ {	0, 0, 1000, 1, perc}, \
/*pid_r_i*/ {	0, 0, 1000, 1, perc}, \
/*pid_r_d*/ {	0, 0, 1000, 1, perc}, \
/*pid_y_p*/ {	0, 0, 1000, 1, perc}, \
/*pid_y_i*/ {	0, 0, 1000, 1, perc}, \
/*pid_y_d*/ {	0, 0, 1000, 1, perc}, \
/*nav_max_angle*/ {	0, 1, 60, 1, none}, \
/*nav_max_acc_lim*/ {	0, 1, 100, 1, none}, \
/*nav_max_accelint*/ {	0, 1, 100, 1, none}, \
/*nav_decel_radius*/ {	0, 0, 50, 1, m}, \
/*nav_decel_radiush*/ {	0, 0, 50, 1, m}, \
/*nav_acc_flt_glob*/ {	0, 0, 1000, 1, none}, \
/*nav_alpha_Pos*/ {	0, 0, 10000, 10, none}, \
/*pid_nav_mislead_on*/ {	0, 0, 1, 1, none}, \
/*pid_nav_mislead_minspeed*/ {	0, 0, 100, 1, mps}, \
/*pid_nav_mislead_mintime*/ {	0, 0, 10, 1, s}, \
/*pid_nav_mislead_filter*/ {	0, 0, 100, 1, perc}, \
/*pid_nav_mislead_mindist*/ {	0, 0, 100, 1, m}, \
/*pid_nav_p*/ {	0, 0, 1000, 1, perc}, \
/*pid_nav_i*/ {	0, 0, 1000, 1, perc}, \
/*pid_nav_d*/ {	0, 0, 1000, 1, perc}, \
/*pid_h_p*/ {	0, 0, 1000, 1, perc}, \
/*pid_h_i*/ {	0, 0, 1000, 1, perc}, \
/*pid_h_d*/ {	0, 0, 2000, 1, perc}, \
/*max_power*/ {	0, 0, 10000, 100, perc}, \
/*idle_power*/ {	0, 0, 3000, 100, perc}, \
/*Agilityfactor*/ {	0, 1, 2, 1, none}, \
/*waitGPSFix*/ {	0, 0, 1, 1, none}, \
/*NoConfigBTM222atStart*/ {	0, 0, 1, 1, none}, \
/*DirectMode*/ {	0, -6, 1, 1, none}, \
/*dsmx_on*/ {	0, 0, 1, 1, none}, \
/*cal_gyro_filter*/ {	0, 1, 32, 1, none}, \
/*cal_acc_filter*/ {	0, 1, 32, 1, none}, \
/*cal_mag_filter*/ {	0, 1, 32, 1, none}, \
/*cal_baro_filter*/ {	0, 1, 1000, 10, none}, \
/*cal_gps_filter*/ {	0, 1, 1000, 10, none}, \
/*cal_gyro_x*/ {	0, -32000, 32000, 1, perc}, \
/*cal_gyro_y*/ {	0, -32000, 32000, 1, perc}, \
/*cal_gyro_z*/ {	0, -32000, 32000, 1, perc}, \
/*cal_acc_x*/ {	0, -32000, 32000, 1, perc}, \
/*cal_acc_y*/ {	0, -32000, 32000, 1, perc}, \
/*cal_acc_z*/ {	0, -32000, 32000, 1, perc}, \
/*cal_mag_auto_inflight*/ {	0, 0, 1, 1, none}, \
/*cal_mag_x*/ {	0, -32000, 32000, 1, perc}, \
/*cal_mag_y*/ {	0, -32000, 32000, 1, perc}, \
/*cal_mag_z*/ {	0, -32000, 32000, 1, perc}, \
/*SwitchPosHold*/ {	0, 0, 12, 1, ch}, \
/*SwPPosHold*/ {	0, 0, 2, 1, ch}, \
/*SwitchHGov*/ {	0, 0, 12, 1, ch}, \
/*SwPHGov*/ {	0, 0, 2, 1, ch}, \
/*SwitchReturnToHome*/ {	0, 0, 12, 1, ch}, \
/*SwPReturnToHome*/ {	0, 0, 2, 1, ch}, \
/*SwitchAux*/ {	0, 0, 12, 1, ch}, \
/*SwPAux*/ {	0, 0, 2, 1, ch}, \
/*PotiP*/ {	0, 0, 12, 1, ch}, \
/*PotiI*/ {	0, 0, 12, 1, ch}, \
/*PotiD*/ {	0, 0, 12, 1, ch}, \
/*madgwick*/ {	0, 0, 2, 1, none}, \
/*a_madg*/ {	0, 1, 100, 1, none}, \
/*P_mahony*/ {	0, 1, 100, 1, none}, \
/*I_mahony*/ {	0, 1, 100, 1, none}, \
/*test_P*/ {	0, 1, 100, 1, none}, \
/*test_I*/ {	0, 1, 100, 1, none}, \
/*test_D*/ {	0, 1, 100, 1, none}, \
/*wind_freq*/ {	0, 0, 100, 5, none}, \
/*wind_ampl*/ {	0, 0, 100, 5, none}, \
/*mag_mis_freq*/ {	0, 0, 100, 5, none}, \
/*mag_mis_ampl*/ {	0, 0, 100, 5, none}, \
/*gyro_freq*/ {	0, 0, 100, 5, none}, \
/*gyro_ampl*/ {	0, 0, 100, 5, none}, \
/*accel_freq*/ {	0, 0, 100, 5, none}, \
/*accel_ampl*/ {	0, 0, 100, 5, none}, \
/*magneto_freq*/ {	0, 0, 100, 5, none}, \
/*magneto_ampl*/ {	0, 0, 100, 5, none}, \
};
#endif

			//Name	Act	Par	Jmp	Parent	Memory
#ifndef MENUE_MENUE_VARDEF
#define MENUE_MENUE_VARDEF \
MenuItem_t m_items[MENUESIZE] = { \
	/* 0*/	{txtFCMSETTINGS,	 0,	 0,	1,	0,	FLASH}, \
	/* 1*/	{txtGYROPID,	 0,	 0,	9,	0,	FLASH}, \
	/* 2*/	{txtNAV,	 0,	 0,	21,	0,	FLASH}, \
	/* 3*/	{txtSETTINGS,	 0,	 0,	48,	0,	FLASH}, \
	/* 4*/	{txtCALIBRATION,	 0,	 0,	58,	0,	FLASH}, \
	/* 5*/	{txtRCCHANNELS,	 0,	 0,	77,	0,	FLASH}, \
	/* 6*/	{txtSAVETOFLASH,	 actSaveToFlash,	 0,	0,	0,	FLASH}, \
	/* 7*/	{txtDIAGMODE,	 act_diag,	 0,	0,	0,	FLASH}, \
	/* 8*/	{txtTEST,	 0,	 0,	89,	0,	FLASH}, \
	/* 9*/	{txtBACK,	 0,	 0,	1,	1,	FLASH}, \
	/* 10*/	{txtMAXINTEGR,	 0,	 &myPar.integrator_limit,	0,	1,	FLASH}, \
	/* 11*/	{txtGOVATMAX,	 0,	 &myPar.govAtMax,	0,	1,	FLASH}, \
	/* 12*/	{txtQGAIN,	 0,	 &myPar.quat_amp,	0,	1,	FLASH}, \
	/* 13*/	{txtSWASHASPOTI,	 actSwashasPot,	 0,	0,	1,	FLASH}, \
	/* 14*/	{txtSWASHP,	 0,	 &myPar.pid_r_p,	0,	1,	FLASH}, \
	/* 15*/	{txtSWASHI,	 0,	 &myPar.pid_r_i,	0,	1,	FLASH}, \
	/* 16*/	{txtSWASHD,	 0,	 &myPar.pid_r_d,	0,	1,	FLASH}, \
	/* 17*/	{txtYAWASPOTI,	 actYawasPot,	 0,	0,	1,	FLASH}, \
	/* 18*/	{txtYAWP,	 0,	 &myPar.pid_y_p,	0,	1,	FLASH}, \
	/* 19*/	{txtYAWI,	 0,	 &myPar.pid_y_i,	0,	1,	FLASH}, \
	/* 20*/	{txtYAWD,	 0,	 &myPar.pid_y_d,	0,	1,	FLASH}, \
	/* 21*/	{txtBACK,	 0,	 0,	2,	2,	FLASH}, \
	/* 22*/	{txtNAVLIMITS,	 0,	 0,	33,	2,	FLASH}, \
	/* 23*/	{txtNAVFILTERS,	 0,	 0,	39,	2,	FLASH}, \
	/* 24*/	{txtNAVCOMPENSATION,	 0,	 0,	42,	2,	FLASH}, \
	/* 25*/	{txtNAVASPOTI,	 actNAVasPot,	 0,	0,	2,	FLASH}, \
	/* 26*/	{txtNAVP,	 0,	 &myPar.pid_nav_p,	0,	2,	FLASH}, \
	/* 27*/	{txtNAVI,	 0,	 &myPar.pid_nav_i,	0,	2,	FLASH}, \
	/* 28*/	{txtNAVD,	 0,	 &myPar.pid_nav_d,	0,	2,	FLASH}, \
	/* 29*/	{txtHASPOTI,	 actHasPot,	 0,	0,	2,	FLASH}, \
	/* 30*/	{txtHP,	 0,	 &myPar.pid_h_p,	0,	2,	FLASH}, \
	/* 31*/	{txtHI,	 0,	 &myPar.pid_h_i,	0,	2,	FLASH}, \
	/* 32*/	{txtHD,	 0,	 &myPar.pid_h_d,	0,	2,	FLASH}, \
	/* 33*/	{txtBACK,	 0,	 0,	22,	22,	FLASH}, \
	/* 34*/	{txtMAXANGLE,	 0,	 &myPar.nav_max_angle,	0,	22,	FLASH}, \
	/* 35*/	{txtMAXACCEL,	 0,	 &myPar.nav_max_acc_lim,	0,	22,	FLASH}, \
	/* 36*/	{txtMAXACCINT,	 0,	 &myPar.nav_max_accelint,	0,	22,	FLASH}, \
	/* 37*/	{txtRADIUSNAV,	 0,	 &myPar.nav_decel_radius,	0,	22,	FLASH}, \
	/* 38*/	{txtRADIUSH,	 0,	 &myPar.nav_decel_radiush,	0,	22,	FLASH}, \
	/* 39*/	{txtBACK,	 0,	 0,	23,	23,	FLASH}, \
	/* 40*/	{txtACCFLTGLOB,	 0,	 &myPar.nav_acc_flt_glob,	0,	23,	FLASH}, \
	/* 41*/	{txtALPHAPOS,	 0,	 &myPar.nav_alpha_Pos,	0,	23,	FLASH}, \
	/* 42*/	{txtBACK,	 0,	 0,	24,	24,	FLASH}, \
	/* 43*/	{txtCOMPON,	 0,	 &myPar.pid_nav_mislead_on,	0,	24,	FLASH}, \
	/* 44*/	{txtMINSPEED,	 0,	 &myPar.pid_nav_mislead_minspeed,	0,	24,	FLASH}, \
	/* 45*/	{txtMINTIME,	 0,	 &myPar.pid_nav_mislead_mintime,	0,	24,	FLASH}, \
	/* 46*/	{txtFILTER,	 0,	 &myPar.pid_nav_mislead_filter,	0,	24,	FLASH}, \
	/* 47*/	{txtMINDISTANCE,	 0,	 &myPar.pid_nav_mislead_mindist,	0,	24,	FLASH}, \
	/* 48*/	{txtBACK,	 0,	 0,	3,	3,	FLASH}, \
	/* 49*/	{txtMAXPOWER,	 0,	 &myPar.max_power,	0,	3,	FLASH}, \
	/* 50*/	{txtIDLEPOWER,	 0,	 &myPar.idle_power,	0,	3,	FLASH}, \
	/* 51*/	{txtAGILITYFACT,	 0,	 &myPar.Agilityfactor,	0,	3,	FLASH}, \
	/* 52*/	{txtWAITGPSFIX,	 0,	 &myPar.waitGPSFix,	0,	3,	FLASH}, \
	/* 53*/	{txtNOBTCFG,	 0,	 &myPar.NoConfigBTM222atStart,	0,	3,	FLASH}, \
	/* 54*/	{txtDIRECTMODE,	 0,	 &myPar.DirectMode,	0,	3,	FLASH}, \
	/* 55*/	{txtSPEKTRUMDSMX,	 0,	 &myPar.dsmx_on,	0,	3,	FLASH}, \
	/* 56*/	{txtBNDSPEKTRUM,	 actBindSPEKTRUM,	 0,	0,	3,	FLASH}, \
	/* 57*/	{txtSTARTBOOTLOADER,	 actStartBootloader,	 0,	0,	3,	FLASH}, \
	/* 58*/	{txtBACK,	 0,	 0,	4,	4,	FLASH}, \
	/* 59*/	{txtFILTERS,	 0,	 0,	71,	4,	FLASH}, \
	/* 60*/	{txtCALIBRATERA,	 actCalibrate,	 0,	0,	4,	FLASH}, \
	/* 61*/	{txtGYROX,	 0,	 &myPar.cal_gyro_x,	0,	4,	FLASH}, \
	/* 62*/	{txtGYROY,	 0,	 &myPar.cal_gyro_y,	0,	4,	FLASH}, \
	/* 63*/	{txtGYROZ,	 0,	 &myPar.cal_gyro_z,	0,	4,	FLASH}, \
	/* 64*/	{txtACCX,	 0,	 &myPar.cal_acc_x,	0,	4,	FLASH}, \
	/* 65*/	{txtACCY,	 0,	 &myPar.cal_acc_y,	0,	4,	FLASH}, \
	/* 66*/	{txtACCZ,	 0,	 &myPar.cal_acc_z,	0,	4,	FLASH}, \
	/* 67*/	{txtMAGCALFLGHT,	 0,	 &myPar.cal_mag_auto_inflight,	0,	4,	FLASH}, \
	/* 68*/	{txtMAGX,	 0,	 &myPar.cal_mag_x,	0,	4,	FLASH}, \
	/* 69*/	{txtMAGY,	 0,	 &myPar.cal_mag_y,	0,	4,	FLASH}, \
	/* 70*/	{txtMAGZ,	 0,	 &myPar.cal_mag_z,	0,	4,	FLASH}, \
	/* 71*/	{txtBACK,	 0,	 0,	59,	59,	FLASH}, \
	/* 72*/	{txtGYROFILTER,	 0,	 &myPar.cal_gyro_filter,	0,	59,	FLASH}, \
	/* 73*/	{txtACCELFILTER,	 0,	 &myPar.cal_acc_filter,	0,	59,	FLASH}, \
	/* 74*/	{txtMAGNETOFLT,	 0,	 &myPar.cal_mag_filter,	0,	59,	FLASH}, \
	/* 75*/	{txtBAROFILTER,	 0,	 &myPar.cal_baro_filter,	0,	59,	FLASH}, \
	/* 76*/	{txtGPSFILTER,	 0,	 &myPar.cal_gps_filter,	0,	59,	FLASH}, \
	/* 77*/	{txtBACK,	 0,	 0,	5,	5,	FLASH}, \
	/* 78*/	{txtSWPOSHOLD,	 0,	 &myPar.SwitchPosHold,	0,	5,	FLASH}, \
	/* 79*/	{txtSWPPOSHOLD,	 0,	 &myPar.SwPPosHold,	0,	5,	FLASH}, \
	/* 80*/	{txtSWHGOV,	 0,	 &myPar.SwitchHGov,	0,	5,	FLASH}, \
	/* 81*/	{txtSWPHGOV,	 0,	 &myPar.SwPHGov,	0,	5,	FLASH}, \
	/* 82*/	{txtSWRTH,	 0,	 &myPar.SwitchReturnToHome,	0,	5,	FLASH}, \
	/* 83*/	{txtSWPRTH,	 0,	 &myPar.SwPReturnToHome,	0,	5,	FLASH}, \
	/* 84*/	{txtSWAUX,	 0,	 &myPar.SwitchAux,	0,	5,	FLASH}, \
	/* 85*/	{txtSWPAUX,	 0,	 &myPar.SwPAux,	0,	5,	FLASH}, \
	/* 86*/	{txtPOTIP,	 0,	 &myPar.PotiP,	0,	5,	FLASH}, \
	/* 87*/	{txtPOTII,	 0,	 &myPar.PotiI,	0,	5,	FLASH}, \
	/* 88*/	{txtPOTID,	 0,	 &myPar.PotiD,	0,	5,	FLASH}, \
	/* 89*/	{txtBACK,	 0,	 0,	8,	8,	FLASH}, \
	/* 90*/	{txtMADGWICK,	 0,	 &myPar.madgwick,	0,	8,	FLASH}, \
	/* 91*/	{txtALPHAMADG,	 0,	 &myPar.a_madg,	0,	8,	FLASH}, \
	/* 92*/	{txtPROPMAH,	 0,	 &myPar.P_mahony,	0,	8,	FLASH}, \
	/* 93*/	{txtINTEGRMAH,	 0,	 &myPar.I_mahony,	0,	8,	FLASH}, \
	/* 94*/	{txtTESTP,	 0,	 &myPar.test_P,	0,	8,	FLASH}, \
	/* 95*/	{txtTESTI,	 0,	 &myPar.test_I,	0,	8,	FLASH}, \
	/* 96*/	{txtTESTD,	 0,	 &myPar.test_D,	0,	8,	FLASH}, \
	/* 97*/	{txtWIND,	 0,	 0,	102,	8,	FLASH}, \
	/* 98*/	{txtMAGMIS,	 0,	 0,	105,	8,	FLASH}, \
	/* 99*/	{txtGYRO,	 0,	 0,	108,	8,	FLASH}, \
	/* 100*/	{txtACCEL,	 0,	 0,	111,	8,	FLASH}, \
	/* 101*/	{txtMAGNETO,	 0,	 0,	114,	8,	FLASH}, \
	/* 102*/	{txtWINDFREQ,	 0,	 &myPar.wind_freq,	0,	97,	FLASH}, \
	/* 103*/	{txtWINDAMPL,	 0,	 &myPar.wind_ampl,	0,	97,	FLASH}, \
	/* 104*/	{txtBACK,	 0,	 0,	97,	97,	FLASH}, \
	/* 105*/	{txtMAGMISFREQ,	 0,	 &myPar.mag_mis_freq,	0,	98,	FLASH}, \
	/* 106*/	{txtMAGMISAMPL,	 0,	 &myPar.mag_mis_ampl,	0,	98,	FLASH}, \
	/* 107*/	{txtBACK,	 0,	 0,	98,	98,	FLASH}, \
	/* 108*/	{txtGYROFREQ,	 0,	 &myPar.gyro_freq,	0,	99,	FLASH}, \
	/* 109*/	{txtGYROAMPL,	 0,	 &myPar.gyro_ampl,	0,	99,	FLASH}, \
	/* 110*/	{txtBACK,	 0,	 0,	99,	99,	FLASH}, \
	/* 111*/	{txtACCELFREQ,	 0,	 &myPar.accel_freq,	0,	100,	FLASH}, \
	/* 112*/	{txtACCELAMPL,	 0,	 &myPar.accel_ampl,	0,	100,	FLASH}, \
	/* 113*/	{txtBACK,	 0,	 0,	100,	100,	FLASH}, \
	/* 114*/	{txtMAGNETOFREQ,	 0,	 &myPar.magneto_freq,	0,	101,	FLASH}, \
	/* 115*/	{txtMAGNETOAMPL,	 0,	 &myPar.magneto_ampl,	0,	101,	FLASH}, \
	/* 116*/	{txtBACK,	 0,	 0,	101,	101,	FLASH}, \
};
#endif

//******** END OF AUTO-GENERATED HEADER DO NOT EDIT!!! *********

