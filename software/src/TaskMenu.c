/*
 * TaskMenu.c
 * Generating the menu
 *
 * Created: 26.05.2013 21:26:38
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
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "TaskMenu.h"
#include "menu/menu.h"
#include "menu/menu_variant.h"
#include "modules/hottv4.h"
#include "modules/types.h"
#include "modules/usart.h"
#include "modules/fcmcp.h"


void TaskMenu(void);
void ProduceMenu(void);

extern uint8_t menu_show_diag;
uint32_t gl_MenuCommand;
uint8_t* gl_Menu_pBuf = 0;
uint8_t gl_MenuSendBT = 0;


void TaskMenu(void)
{
	OS_WaitTicks(OSALM_MENUWAIT,10); 
	
	while(1)
	{
		
		OS_WaitTicks(OSALM_MENUWAIT,100);
		if(fcmcp_getStreamState()== fcmcp_stream_menu)
		{
			ProduceMenu();
			//OS_WaitTicks(OSALM_MENUWAIT,10);
		}
		
		while(fcmcp_getStreamState()== fcmcp_stream_parl)
		{
			char resp[sizeof(Parinfo_t)+6+3] = {"---PAR"};
			Parinfo_t pi;
			if(PAR_SendNextItem(&pi))
			{
				for (uint32_t i = 0;i<sizeof(Parinfo_t);i++)
				{
					resp[i+6]= ((char*)&pi)[i];
				}
			
				strncpy(&resp[sizeof(Parinfo_t)+6],"~~~",3);
				USART_Send(0,(uint8_t*)&resp,sizeof(Parinfo_t)+9); // needs xxx ms
			}
			OS_WaitTicks(OSALM_MENUWAIT,10);
		}
		
		//OS_WaitEvent(OSEVT_MENUWAKEUP);
		// concept:
		// wait for menue event
		
		// handle menue task
		// prepare menue buffer output
		// send out to "customer"

		

	}
}

void TMNU_SetPtr(uint8_t* ptr)
{
	gl_Menu_pBuf = ptr;
}

void TMNU_TriggerMenu(uint32_t trigger,uint8_t SendBT)
{
	if(gl_MenuCommand != 0 || gl_Menu_pBuf == NULL)
		return;
	gl_MenuCommand = trigger;
	gl_MenuSendBT = SendBT;
	fcmcp_setStreamState(fcmcp_stream_menu);
	//OS_SetEventFromISR(OSTSK_MENU,OSEVT_MENUWAKEUP);
}


void ProduceMenu(void)
{
	if( gl_Menu_pBuf == NULL)
	return;
		
	switch(gl_MenuCommand)
	{
		case 0:
		case '0':
			// do nothing here, as this means that no button is pressed.
			break;
		case 'P':
			button_FastPlus();
			break;
		case 'p':
			button_Plus();
			break;
		case 'M':
			button_FastMinus();
			break;
		case 'm':
			button_Minus();
			break;
		case 'E':
			if(menu_show_diag == 1)
			{
				menu_show_diag = 0;
			}
			else
			{
				menu_select();
			}
			break;
		default:
			break;
	}
			
			
	/************************************************************************/
	/* The menu interface                                                   */
	/************************************************************************/
			
	memset(gl_Menu_pBuf, 0, HOTT_TEXTMODE_MSG_TEXT_LEN);
	if(menu_show_diag == 0)
	{
		menu_setPtr((char *)&gl_Menu_pBuf[0]);
		menu_show(); // do after the key handling, so the keys are already processed.
			
		GetPotiTxt((char *)&gl_Menu_pBuf[21*7]);	
	}
	else
	{
		GetDiagTxt((char *)&gl_Menu_pBuf[0]);
			
	}
		
		
			
	// clean up the string before sending. otherwise the Graupner transmitter displays garbage in '0' spaces.
	for (int i =0; i< HOTT_TEXTMODE_MSG_TEXT_LEN-1;i++)
	{
		if (gl_Menu_pBuf[i] & 0x80 )
		{
			gl_Menu_pBuf[i] = gl_Menu_pBuf[i] & 0x7f;
		}
		if (gl_Menu_pBuf[i] == 0)
		{
			gl_Menu_pBuf[i] = ' ';
		}
	}

			
			
			
		
	if(gl_MenuSendBT) // request over BT
	{
		// send message back
		strncpy((char*)&gl_Menu_pBuf[HOTT_TEXTMODE_MSG_TEXT_LEN],"~~~",3);
		USART_Send(0,gl_Menu_pBuf-6,9+HOTT_TEXTMODE_MSG_TEXT_LEN+1);
	}
		
	gl_MenuCommand = 0; // set back to idle = handled
	
		
}