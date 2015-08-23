/*
 * This is the Menu interface for FCM
 *
 * (c) 2012-2015 Fabian Huslik
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
//#include <stdio.h> // sprintf in there is POISON!!
#include <string.h>
#include "menu.h"
#include "menu_variant.h" 
#include "modules/hottv4.h"
#include "modules/printf.h"
#include "modules/ParFlash.h"
#include "../version.h"
#include "servo_in.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "modules/servo_out.h"
#include "modules/GPS.h"
#include "modules/vector.h"
#include "modules/SPEKTRUM.h"
#include "TaskNavi.h"
#include "modules/fcmcp.h"
#include "modules/types.h"
#include "modules/usart.h"
#include "../config.h"
#include "modules/NAV.h"
#include "TaskControl.h"


#include "modules/types.h" // for myTWI.h only

//extern vars
extern volatile Menue_t MyMenue; // in menu.c!! // todo access

static volatile char* menuTargetPtr = NULL;  // todo access

static volatile uint8_t PotiSetting = 0;
volatile uint8_t g_CalibrationRequest;

int16_t l_pot1,l_pot2,l_pot3; // for display the poti settings in idle

//******** INSERT INTO C FILE *********
// Text definitions
MENUE_TEXT_VARDEF

// Parameter definitions
MENUE_PARAM_VARDEF
// Menue definitions
MENUE_MENUE_VARDEF

//******** INSERT INTO C FILE *********

#if MAX_ITEM_NAME_CHARLENGTH > 19

#error "a name will not fit into the display!"

#endif


// ************* LCD interface

void menu_draw_header(char* menu_header)
{
	if(menuTargetPtr == NULL) return;
	strcpy((char*)(menuTargetPtr), menu_header);
	//hott_invert_all_chars((char*)(menuTargetPtr));
}

/************************************************************************
char * item_name	:	string name of item as displayed
Parameter_t* par	:	pointer to a parameter (if any)
uint8_t lcd_pos		:	display line to write to                                                      
************************************************************************/
void menu_draw_unselected_items(char *item_name, Parameter_t* par, uint8_t lcd_pos)
{
	if(menuTargetPtr == NULL) return;
	
	uint8_t offs = (lcd_pos+1)*21+1; // 21 chars per line plus one for the '>'
	
	//delete old value
		memset((void*)&menuTargetPtr[offs],' ',21);
	//draw menu item
	if (par != NULL)
	{
		sprintf((char*)&menuTargetPtr[offs],"%s : %4d",item_name, par->sValue);
	}
	else
	{
		sprintf((char*)&menuTargetPtr[offs],"%s",item_name);
	}
	menuTargetPtr[offs-1]= ' '; // no '>'
}

extern void menu_draw_selected_item(char *item_name, Parameter_t* par, uint8_t lcd_pos)
{
	if(menuTargetPtr == NULL) return;
		
	uint8_t offs = (lcd_pos+1)*21+1; // 21 chars per line plus one for the '>'
		
	//delete old value
	memset((void*)&menuTargetPtr[offs],' ',21);
	//draw menu item
	if (par != NULL)
	{
		sprintf((char*)&menuTargetPtr[offs],"%s : %4d",item_name, par->sValue);
	}
	else
	{
		sprintf((char*)&menuTargetPtr[offs],"%s",item_name);
	}
	menuTargetPtr[offs-1]= '>'; // selected line
}

void menu_draw_selected_parameter(char *item_name, Parameter_t* par, uint8_t lcd_pos)
{
	// same, name preceeded with '>' and value inverted.
	if(menuTargetPtr == NULL) return;
		
	uint8_t offs = (lcd_pos+1)*21+1; // 21 chars per line plus one for the '>'
		
	//delete old value
	memset((void*)&menuTargetPtr[offs],' ',21);
	//draw menu item
	if (par != NULL)
	{
		sprintf((char*)&menuTargetPtr[offs],"%s : %4d",item_name, par->sValue);
		hott_invert_chars((char*)&menuTargetPtr[offs]+strlen(item_name)+3,4);
	}
	else
	{
		sprintf((char*)&menuTargetPtr[offs],"%s",item_name);
	}
	menuTargetPtr[offs-1]= '*'; // selected line
}

void menu_del_menuitems(void)//menu plane is changed
{
	//delete menu items
	memset((void*)(menuTargetPtr+21), ' ', MENU_ITEMS_PER_PAGE*21); 
}

void menu_draw_groupposition(uint8_t itemnr, uint8_t groupitems)
{
	itemnr; // kill a warning and produce another one ;-)
	groupitems; // kill a warning and produce another one ;-)
}

/************************************************************************/
/* Set pointer for the text-mode                                        */
/************************************************************************/
void menu_setPtr(char* ptr)
{
	menuTargetPtr = ptr;
}




//******************* Actions



#define COMPILE_TIME_ASSERT(expr)   \
char TheAssertIsFAILED[expr]
// gives an array of length 0 if failed.

void actCalibrate (void)
{
	g_CalibrationRequest = 1;
	PotiSetting = 100; // display DONE
}

void actSaveToFlash (void)
{
	int16_t pardata_raw[MENUE_PARCOUNT];
	int16_t* pdata = (int16_t*)(&myPar); // todo 	Parameter_t* pdata = (Parameter_t*)(&myPar); ?
	
	// make array of values only...
	for (int i = 0; i < MENUE_PARCOUNT;i++)
	{
		pardata_raw[i] = *pdata;
		pdata += (sizeof(Parameter_t)/2); // 2 byte ptr
	}
	
	// prevent unintentional running of motors:
	OS_PREVENTSCHEDULING;
	servo_out_zero();
	
	ParFlash_save((uint8_t*)&pardata_raw,MENUE_PARCOUNT*2,PARAMETERVERSION);
	OS_ALLOWSCHEDULING;
	
	#if MENUE_PARCOUNT*2+8 >= AVR32_FLASHC_USER_PAGE_SIZE
		#error "too many parameters"
	#endif
	
	PotiSetting = 100; // display DONE
	
}


void actSwashasPot (void)
{
	PotiSetting = 1;
}

void actYawasPot (void)
{
	PotiSetting = 2;
}

void actHasPot (void)
{
	PotiSetting = 3;
}

void actNAVasPot (void)
{
	PotiSetting = 4;
}

void actBindSPEKTRUM(void)
{
	SPEKTRUM_Bind();
	PotiSetting = 100; // display DONE

}

void actStartBootloader(void)
{
	// jump into bootloader
	AVR32_PM.gplp[0] = 0xB00710AD; // magic word for BL "bootload"
	wdt_reset_mcu(); // only works, if wd is enabled
}

void UpdatePotsFromTX(void) // update the parameters from the potis ; 
{
	// todo some pointer magic for smalle code?
	int32_t cmd_pot1=0,cmd_pot2=0,cmd_pot3=0;
	
	if(myPar.PotiP.sValue >0 && servo_in_get_ext_channel(myPar.PotiP.sValue,&cmd_pot1)==0)
	{
		switch (PotiSetting)
		{
			case 0:
			l_pot1 = cmd_pot1/POTDIVISOR200;
			break;
			case 1:
			myPar.pid_r_p.sValue = cmd_pot1/POTDIVISOR200;
			break;
			case 2:
			myPar.pid_y_p.sValue = cmd_pot1/POTDIVISOR200;
			break;
			case 3:
			myPar.pid_h_p.sValue = cmd_pot1/POTDIVISOR200;
			break;
			case 4:
			myPar.pid_nav_p.sValue = cmd_pot1/POTDIVISOR100;
			break;
			default:
			break;
		}
	}
	if(myPar.PotiI.sValue >0 && servo_in_get_ext_channel(myPar.PotiI.sValue,&cmd_pot2)==0)
	{
		switch (PotiSetting)
		{
			case 0:
			l_pot2 = cmd_pot2/POTDIVISOR200;
			break;
			case 1:
			myPar.pid_r_i.sValue = cmd_pot2/POTDIVISOR200;
			break;
			case 2:
			myPar.pid_y_i.sValue = cmd_pot2/POTDIVISOR200;
			break;
			case 3:
			myPar.pid_h_i.sValue = cmd_pot2/POTDIVISOR200;
			break;
			case 4:
			myPar.pid_nav_i.sValue = cmd_pot2/POTDIVISOR100;
			break;
			default:
			break;
		}
	}
	if(myPar.PotiD.sValue >0 && servo_in_get_ext_channel(myPar.PotiD.sValue,&cmd_pot3)==0)
	{
		switch (PotiSetting)
		{
			case 0:
			l_pot3 = cmd_pot3/POTDIVISOR200;
			break;
			case 1:
			myPar.pid_r_d.sValue = cmd_pot3/POTDIVISOR200;
			break;
			case 2:
			myPar.pid_y_d.sValue = cmd_pot3/POTDIVISOR200;
			break;
			case 3:
			myPar.pid_h_d.sValue = cmd_pot3/POTDIVISOR200;
			break;
			case 4:
			myPar.pid_nav_d.sValue = cmd_pot3/POTDIVISOR200;
			break;
			default:
			break;
		}
	}
}



void GetPotiTxt( char* ptxt ) 
{
	static uint8_t done=0;
	switch (PotiSetting)
	{
		case 0:
			sprintf(ptxt,"idle %3d %3d %3d",l_pot1,l_pot2,l_pot3);
			break;
		case 1:
			sprintf(ptxt,"Swa P%3d I%3d D%3d",myPar.pid_r_p.sValue,myPar.pid_r_i.sValue,myPar.pid_r_d.sValue);
			break;
		case 2:
			sprintf(ptxt,"Yaw P%3d I%3d D%3d",myPar.pid_y_p.sValue,myPar.pid_y_i.sValue,myPar.pid_y_d.sValue);	
			break;
		case 3:
			sprintf(ptxt,"H-G P%3d I%3d D%3d",myPar.pid_h_p.sValue,myPar.pid_h_i.sValue,myPar.pid_h_d.sValue);
			break;
		case 4:
			sprintf(ptxt,"NAV P%3d I%3d D%3d",myPar.pid_nav_p.sValue,myPar.pid_nav_i.sValue,myPar.pid_nav_d.sValue);
			break;
		case 100:
			done = 10; PotiSetting = 101;
		default:
			sprintf(ptxt,"** DONE **");
			if(done-- == 0)
				PotiSetting = 0;
			break;
	}
}


//#define SHOWSTACKS
//#define SHOWCTL
//#define SHOWGYRO
#define SHOWNAV

void GetDiagTxt( char* ptxt )
{
	//									 "---------------------"
	//									 " FCM 2.1.0 DIAG "
	sprintf(ptxt,"FCM "VERSION_STRING );
	//hott_invert_chars(ptxt,21);
	//sprintf(&ptxt[1*21],"SW PID %d %d %d ", myPar.pid_r_p.sValue,myPar.pid_r_i.sValue,myPar.pid_r_d.sValue);
	uint32_t m,s,ms;
	ms = OS_GetTicks();
	s = ms/1000;
	m = s / 60;
	s = s % 60;
	ms = ms % 1000;
	
	
	#ifdef SHOWSTACKS
		static uint16_t t1,t2,t3,t4,t5,t6,t7;
		static uint8_t scc;
		scc++;
		if(scc == 10) // takes too long for every task at once
		{
			scc = 0;
			t1 = OS_GetUnusedStack(OSTSK_CONTROL);
			t2 = OS_GetUnusedStack(OSTSK_TWI);
			t3 = OS_GetUnusedStack(OSTSK_COMM2);
			t4 = OS_GetUnusedStack(OSTSK_COMM0);
			t5 = OS_GetUnusedStack(OSTSK_LED);
			t6 = OS_GetUnusedStack(OSTSK_MENU);
			t7 = OS_GetUnusedStack(OSTSK_NAVI);
		}
	
		sprintf(&ptxt[2*21],"Stacks: NAV:%d ",t7);
		sprintf(&ptxt[3*21],"CTL:%d TWI:%d ",t1,t2);
		sprintf(&ptxt[4*21],"CM2:%d CM0:%d ",t3,t4);
		sprintf(&ptxt[5*21],"LED:%d MNU:%d ",t5,t6);
	#else
		#ifdef SHOWCTL
			extern int32_t debug_xc, debug_yc, debug_zc, debug_ac, debug_xg, debug_yg, debug_zg, debug_ag;
	
			sprintf(&ptxt[2*21],"x: %d y:%d ", debug_xc, debug_yc);
			sprintf(&ptxt[3*21],"z: %d a:%d ", debug_zc, debug_ac);
			sprintf(&ptxt[4*21],"x: %d y:%d ", debug_xg, debug_yg);
			sprintf(&ptxt[5*21],"z: %d a:%d ", debug_zg, debug_ag);
		#else
			#ifdef SHOWGYRO
				extern int32_t debug_GY_NoMeasure, debug_Gy_INcomplete, debug_Gy_complete;
				extern uint32_t debug_RXINTERR,debug_RXERR,debug_Gy_RECONFIG,debug_SPI_ovres;
				extern int32_t debug_TWI_CountOfMisReads;
				sprintf(&ptxt[1*21],"Time: %3d:%2d:%3d", m,s,ms);
				sprintf(&ptxt[2*21],"No Gyro:%d REC:%d", (uint16_t)debug_GY_NoMeasure, (uint16_t)debug_Gy_RECONFIG);
				sprintf(&ptxt[3*21],"Inc :%d CPL:%d",(uint16_t)debug_Gy_INcomplete,(uint16_t)debug_Gy_complete);
				sprintf(&ptxt[4*21],"RX-I:%d RX-B:%d ",(uint16_t)debug_RXINTERR,(uint16_t)debug_RXERR);
				sprintf(&ptxt[5*21],"ovr:%d ",debug_SPI_ovres);
			#else
				#ifdef SHOWTWI
					extern int32_t debug_TWI_CountOfMisReads;
					extern uint32_t  debug_TWI_noTxComp, debug_TWI_TxComp, debug_TWI_NoRead;
					extern twitrnsf_t debug_TWI_lastTrnsf;
				    sprintf(&ptxt[1*21],"Time: %3d:%2d:%3d", m,s,ms);
					sprintf(&ptxt[2*21],"tnc:%d tc:%d", (uint16_t)debug_TWI_noTxComp, (uint16_t)debug_TWI_TxComp);
					sprintf(&ptxt[3*21],"mis:%d nak:%d",(uint16_t)debug_TWI_CountOfMisReads, (uint16_t)debug_TWI_NoRead);
					sprintf(&ptxt[4*21],"tim:%d ",(uint16_t)debug_TWI_lastTrnsf.lastTime);

				#else
					#ifdef SHOWNAV
						sprintf(&ptxt[1*21],"Cset: %d %d %d", (int16_t)(NAV_info.setcmd.x*100), (int16_t)(NAV_info.setcmd.y*100), (int16_t)(NAV_info.setcmd.z*100));
						sprintf(&ptxt[2*21],"Pact: %d %d %d", (int16_t)NAV_info.Pos.x, (int16_t)NAV_info.Pos.y, (int16_t)NAV_info.Pos.z);
						sprintf(&ptxt[3*21],"Pset: %d %d %d", (int16_t)NAV_info.Set.x, (int16_t)NAV_info.Set.y, (int16_t)NAV_info.Set.z);
						sprintf(&ptxt[4*21],"Hdg :%d Trg:%d",(int16_t)IMUdata.mag_heading_deg,(int16_t)NAV_info.TrgHeading_deg);
						sprintf(&ptxt[5*21],"Sat %d Dst %dm", GPS_GetNumSats(),(int16_t)NAV_info.TrgDist_m);
						sprintf(&ptxt[6*21],"Rot: %d %d %d", (int16_t)NAV_info.att_deg.x, (int16_t)NAV_info.att_deg.y, (int16_t)NAV_info.att_deg.z);
						sprintf(&ptxt[7*21],"RSet: %d %d %d", (int16_t)NAV_info.attSet_deg.x, (int16_t)NAV_info.attSet_deg.y, (int16_t)NAV_info.attSet_deg.z);  
						//sprintf(&ptxt[4*21],"RX-I:%d RX-B:%d ",(uint16_t)debug_RXINTERR,(uint16_t)debug_RXERR);
						//sprintf(&ptxt[5*21],"ovr:%d ",debug_SPI_ovres);

					#else
						//*** standard view
						bool swHGov = servo_in_get_ext_channel_switch(myPar.SwitchHGov.sValue,myPar.SwPHGov.sValue);
						bool swPosHold = servo_in_get_ext_channel_switch(myPar.SwitchPosHold.sValue,myPar.SwPPosHold.sValue);
						bool swRTH = servo_in_get_ext_channel_switch(myPar.SwitchReturnToHome.sValue,myPar.SwPReturnToHome.sValue);
						bool swAUX = servo_in_get_ext_channel_switch(myPar.SwitchAux.sValue,myPar.SwPAux.sValue);
						
						int32_t cmd_pot_P;
						servo_in_get_ext_channel(myPar.PotiP.sValue, &cmd_pot_P);
						int32_t cmd_pot_I;
						servo_in_get_ext_channel(myPar.PotiI.sValue, &cmd_pot_I);
						int32_t cmd_pot_D;
						servo_in_get_ext_channel(myPar.PotiD.sValue, &cmd_pot_D);
						extern uint32_t debug_RunTimeMax;
				
						int32_t ch5val; servo_in_get_ext_channel(5,&ch5val);
						int32_t ch6val; servo_in_get_ext_channel(6,&ch6val);
						int32_t ch7val; servo_in_get_ext_channel(7,&ch7val);
						int32_t ch8val; servo_in_get_ext_channel(8,&ch8val);
						int32_t ch9val; servo_in_get_ext_channel(9,&ch9val);
						int32_t ch10val; servo_in_get_ext_channel(10,&ch10val);
						int32_t ch11val; servo_in_get_ext_channel(11,&ch11val);
						int32_t ch12val; servo_in_get_ext_channel(12,&ch12val);
						sprintf(&ptxt[1*21],"Time: %3d:%2d:%3d", m,s,ms);
						sprintf(&ptxt[2*21],"%d %d %d %d",ch5val,ch6val,ch7val,ch8val);
						sprintf(&ptxt[3*21],"%d %d %d %d",ch9val,ch10val,ch11val,ch12val);

						sprintf(&ptxt[4*21],"RTH:%d Pos:%d HG:%d A:%d", (uint16_t)swRTH, (uint16_t)swPosHold,(uint16_t)swHGov, (uint16_t)swAUX);
						sprintf(&ptxt[5*21],"Pot %3d %3d %3d",cmd_pot_P,cmd_pot_I,cmd_pot_D);
						//sprintf(&ptxt[6*21],"RunTime: %d",(uint16_t)debug_RunTimeMax);
						debug_RunTimeMax = 0;
				
						sprintf(&ptxt[6*21],"Sat %d Dst %dm %d ", GPS_GetNumSats(),NAV_info.TrgDist_m, (int16_t)NAV_info.TrgHeading_deg);
						sprintf(&ptxt[7*21],"(c) 2015 huslik.net");
					#endif
				#endif	
			#endif
		#endif
	#endif
	
	//1:1234 2:1234 3:1234
	//					 123456789012345678901
	
}


bool PAR_SendNextItem (Parinfo_t* pi)
{
	static int i=0;
	// parse menu
	if (i<MENUESIZE)
	{
		pi->ID = i;
		pi->Parent = m_items[i].ucParent;
		strcpy(pi->text,m_items[i].strName);
		if(m_items[i].pParam != NULL)
		{
			// this is a parameter
			pi->value = m_items[i].pParam->sValue;
			pi->upper = m_items[i].pParam->sUpperLimit;
			pi->lower = m_items[i].pParam->sLowerLimit;
		}
		else
		{
			pi->value = 0;
			pi->upper = 0;
			pi->lower = 0;
		}
		i++;
		return true;
	}
	else
	{
		i=0;
		fcmcp_setStreamState(fcmcp_stream_idle);
		return false;
	}
	

}

void menue_setParVal(uint8_t id,int16_t val)
{
	if(id < MENUESIZE)
	{
		if (m_items[id].pParam != NULL)
		{
			if (val <= m_items[id].pParam->sUpperLimit && val >= m_items[id].pParam->sLowerLimit)
			{
				m_items[id].pParam->sValue = val;
			}
		}
	}

}

void menue_sendpar(uint8_t item)
{
	#if(COM0_FCMCP == 1) // do not send with HoTT
	
	char resp[sizeof(Parinfo_t)+6+3] = {"---PAR"};
	Parinfo_t pi;
	
	pi.ID = item;
	pi.Parent = m_items[item].ucParent;
	strcpy(pi.text,m_items[item].strName);
	if(m_items[item].pParam != NULL)
	{
		// this is a parameter
		pi.value = m_items[item].pParam->sValue;
		pi.upper = m_items[item].pParam->sUpperLimit;
		pi.lower = m_items[item].pParam->sLowerLimit;
	}
	else
		return;
		
	for (uint32_t i = 0;i<sizeof(Parinfo_t);i++)
	{
		resp[i+6]= ((char*)&pi)[i];
	}
		
	strncpy(&resp[sizeof(Parinfo_t)+6],"~~~",3);
	USART_Send(0,(uint8_t*)&resp,sizeof(Parinfo_t)+9); // needs xxx ms // fixme modularize

	OS_WaitTicks(OSALM_MENUWAIT,10);
	#endif
}


	