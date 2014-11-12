 /*
    This is the Menu - an universal embedded menue system, fully configurable.

	(c) 2010-2014 Jörg Schmidt & Fabian Huslik
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <asf.h>
#include "menu.h"
#include "menu_variant.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"

extern MenuItem_t m_items[]; // todo access?

volatile Menue_t MyMenue = MENUEINIT;

void GetSubMenuCount(uint8_t *Size, uint8_t *StartIndex)
{
	uint8_t i, ResultStartIndex=0, ResultSize=0;
	for(i=m_items[MyMenue.gucSelectedItem].ucParent+1;i<MENUESIZE;i++)
	{
		if(m_items[i].ucParent == m_items[MyMenue.gucSelectedItem].ucParent)
			{
			if(ResultStartIndex == 0) ResultStartIndex = i;	
			ResultSize++;								
			}
	}
	//Assign values
	*Size = ResultSize;
	*StartIndex = ResultStartIndex;
}

void AdjustStartPos(uint8_t *Size, uint8_t *StartIndex)
{	
	if(*Size>MENU_ITEMS_PER_PAGE)
	{
		//bottom screen
		if((MyMenue.gucSelectedItem-*StartIndex)>=(*Size-(MENU_ITEMS_PER_PAGE/2)))
		{
			*StartIndex=MyMenue.gucSelectedItem-(MENU_ITEMS_PER_PAGE-(*Size-(MyMenue.gucSelectedItem-*StartIndex)));
		}
		else
		{
			//middle screen
			if((MyMenue.gucSelectedItem-*StartIndex)>=(MENU_ITEMS_PER_PAGE/2))
			{
				*StartIndex=MyMenue.gucSelectedItem-(MENU_ITEMS_PER_PAGE/2);
			}
		}
	}

}


void menu_init(void)
{
	uint8_t i;
	uint8_t SubMenuGroupSize, StartIndex;

	//get and check array inidices
	for(i=0;i<MENUESIZE;i++)
	{
		if(m_items[i].ucParent < MENUESIZE )
		{
		}
		else
		{
		//putt
		}
	}
	//set actual limits for UI
	GetSubMenuCount(&SubMenuGroupSize, &StartIndex);

	HandOverValueToUI(StartIndex, SubMenuGroupSize+StartIndex-1, StartIndex,1);

}

void menu_show(void)
{	
	uint8_t SubMenuGroupSize=0, StartIndex=1, i=0, LCDPos=0;
	uint8_t items_per_page;
	char StrTmp[MAX_ITEM_NAME_CHARLENGTH+1]; // tailing zero!!!
	static uint8_t LastMenuIndex=0;
	static uint8_t SelParLCDPos = 0;
	static uint8_t OldParentID = 255;
	static uint16_t OldParameterValue=255;
	static uint16_t NewParameterValue;
	
	uint8_t MenuMode,SelectedItem;

	//get new value from UI
	OS_PREVENTSCHEDULING; // for MyMenue.MenuMode AND MyMenue.gucSelectedItem
	MenuMode = MyMenue.MenuMode;
	SelectedItem = MyMenue.gucSelectedItem;
	OS_ALLOWSCHEDULING;


	if(MenuMode)	
	{
		SelectedItem = (uint8_t)GetvalueFromUI(); // menue runs no more backwards.
		OS_PREVENTSCHEDULING;
		MyMenue.gucSelectedItem = SelectedItem;
		OS_ALLOWSCHEDULING;
	}
	else	// done later
	{


	}


	if(LastMenuIndex!=SelectedItem || MyMenue.ShowMenu || MENU_DRAWALWAYS)//new menu item is selected
	{
		//save new index
		LastMenuIndex=SelectedItem;
		//get array start index of the submenu items
		GetSubMenuCount(&SubMenuGroupSize,&StartIndex);
		//send values to UI
		//calculate items_per_page
		if(SubMenuGroupSize>MENU_ITEMS_PER_PAGE) 
		{
			items_per_page = MENU_ITEMS_PER_PAGE;
		}
		else
		{
			items_per_page = SubMenuGroupSize;
		}
		//write menu header
		if(OldParentID!=m_items[SelectedItem].ucParent || MyMenue.ShowMenu || MENU_DRAWALWAYS)
		{
			OldParentID=m_items[SelectedItem].ucParent;
			if((m_items[m_items[SelectedItem].ucParent].ucSettings&0x03) == SRAM)
			{
				menu_draw_header(m_items[m_items[SelectedItem].ucParent].strName);
			}
			else
			{	
				strcpy_P(StrTmp,m_items[m_items[SelectedItem].ucParent].strName);
				menu_draw_header(StrTmp);
			}
			//del old menu items
			menu_del_menuitems();
		}
		//reset show menu
		MyMenue.ShowMenu = 0;
		//write position information
		menu_draw_groupposition(SelectedItem, SubMenuGroupSize);
		//Adjust Start index
		AdjustStartPos(&SubMenuGroupSize, &StartIndex);
		//write menu items
		LCDPos = 0;
		for(i=StartIndex;i<StartIndex+items_per_page;i++)
		{
			if(i==SelectedItem) //draw item
			{
				if(MenuMode)
				{
					//save pos
					SelParLCDPos = LCDPos;
					//draw selected item
					if((m_items[i].ucSettings&0x03) == SRAM) // todo maybe remove the variant??
					{
						menu_draw_selected_item(m_items[i].strName, m_items[i].pParam, LCDPos);
					}
					else
					{	
						strcpy_P(StrTmp,m_items[i].strName);
						menu_draw_selected_item(StrTmp, m_items[i].pParam, LCDPos);
					}
				}
				else // it has to be the active parameter.
				{
					NewParameterValue=GetvalueFromUI();		
					if(NewParameterValue!=OldParameterValue || MyMenue.ShowParameter || MENU_DRAWALWAYS)//new menu item is selected
					{
						MyMenue.ShowParameter = 0;
						//save value
						m_items[MyMenue.gucSelectedItem].pParam->sValue = NewParameterValue;
						OldParameterValue=NewParameterValue;
					
						//draw selected item
						if((m_items[SelectedItem].ucSettings & 0x03) == SRAM)
						{
							menu_draw_selected_parameter(m_items[SelectedItem].strName, m_items[SelectedItem].pParam, SelParLCDPos);
						}
						else
						{	
							strcpy_P(StrTmp,m_items[SelectedItem].strName);
							menu_draw_selected_parameter(StrTmp, m_items[SelectedItem].pParam, SelParLCDPos);
						}
		
					}					
					
				}
			}
			else
			{
				if((m_items[i].ucSettings&0x03) == SRAM)
				{
					menu_draw_unselected_items(m_items[i].strName, m_items[i].pParam, LCDPos);
				}
				else
				{	
					strcpy_P(StrTmp,m_items[i].strName);
					menu_draw_unselected_items(StrTmp, m_items[i].pParam, LCDPos);
				}
			}
		LCDPos++;
		}
	}
}

void menu_select(void)
{
	uint8_t SubMenuGroupSize=0, StartIndex=0;
	OS_MutexGet(OSMTX_COMM);//OS_PREVENTSCHEDULING; // for MyMenue.MenuMode AND MyMenue.gucSelectedItem / Parameter // fixme counterpart missing!
	
	if(m_items[MyMenue.gucSelectedItem].ucJumpTrg)//jump to target menu item
	{
		//assign jump target
		MyMenue.gucSelectedItem = m_items[MyMenue.gucSelectedItem].ucJumpTrg;
		//get array start index of the submenu items
		GetSubMenuCount(&SubMenuGroupSize, &StartIndex);
		//send values to UI
		HandOverValueToUI(MyMenue.gucSelectedItem, StartIndex+SubMenuGroupSize-1, StartIndex,1);
	}	
	else if(m_items[MyMenue.gucSelectedItem].pParam)//toggle parameter edit mode
	{
		if(!MyMenue.MenuMode)
		{	
			//save parameter
			m_items[MyMenue.gucSelectedItem].pParam->sValue=GetvalueFromUI();
			
			// send par over active protocol
			menue_sendpar(MyMenue.gucSelectedItem);
			
			//get array start index of the submenu items
			GetSubMenuCount(&SubMenuGroupSize,&StartIndex);
			//send values to UI
			HandOverValueToUI(MyMenue.gucSelectedItem, StartIndex+SubMenuGroupSize-1, StartIndex,1);
			//parameter deactivate
			MyMenue.MenuMode = 1;
			MyMenue.ShowMenu = 1; //init menu view

		}
		else
		{	
			//send parameter values and limits to UI
			HandOverValueToUI(	m_items[MyMenue.gucSelectedItem].pParam->sValue,
								m_items[MyMenue.gucSelectedItem].pParam->sUpperLimit,
								m_items[MyMenue.gucSelectedItem].pParam->sLowerLimit,
								m_items[MyMenue.gucSelectedItem].pParam->sStepSize
								);

			//parameter active
			MyMenue.MenuMode = 0;

			MyMenue.ShowParameter = 1; //init parameter view
		}
	}
	else if(m_items[MyMenue.gucSelectedItem].pAction)//execute function
	{	
		//get array start index of the submenu items
		GetSubMenuCount(&SubMenuGroupSize, &StartIndex);
		//send values to UI
		HandOverValueToUI(MyMenue.gucSelectedItem, StartIndex+SubMenuGroupSize-1, StartIndex,1);
		MyMenue.ShowMenu = 1; //init menu view
		//execute function
		m_items[MyMenue.gucSelectedItem].pAction();
		// do nothing after function, as it might have changed the menu status or content intentionally.
	}
	else
	{
//		asm("break");
	}
	OS_MutexRelease(OSMTX_COMM);//OS_ALLOWSCHEDULING;
}


