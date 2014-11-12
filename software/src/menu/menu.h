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


#ifndef MENU_H_DEFINED
#define MENU_H_DEFINED

typedef struct Menue_tag
{
	uint8_t gucSelectedItem;
	uint8_t ShowParameter;
	uint8_t ShowMenu;
	uint8_t MenuMode;
}Menue_t;

#define MENUEINIT {1,0,0,1}


typedef struct Parameter_tag // do not change types or order, generator uses this type!
{
	int16_t		sValue;			//Parameter value
	int16_t		sLowerLimit;	//Parameter Lower Limit
	int16_t		sUpperLimit;	//Parameter Upper Limit
	int16_t		sStepSize;		//Parameter Step size
	uint16_t		sType;		//Parameter type (Voltage, Current...)	// alignment!!!
}Parameter_t;


typedef struct MenuItemName_tag
{
	//must be initialized by user
	char* 			strName;		//Name
}
MenuItemName_t;

typedef struct MenuItem_tag
{
	char* 			strName;		//Name
	void(*			pAction)(void);	//Function to execute
	Parameter_t*	pParam;		//Parameter to change
	uint8_t			ucJumpTrg; 		//jump target
	uint8_t			ucParent;		//Parent item (0 = root)
	uint8_t 		ucSettings;		//menu item settings 
									//0bxxxx abcc 
									//		 ||	|_ 	cc 	menu item str location Flash(0), RAM(1) or EEPROM(2)
									//		 ||___	b 	menu item is selected
									//		 |____  a	paramter edit mode active (maybe a== b ?)
}
MenuItem_t;

//Errorcode
#define INFINITE_LOOP 	1			//A infinite loop exists in the menu structure
#define WRONG_SUB_UCID 	2			//The ID of the sub menu item is wrong
#define WRONG_PREV_UCID 3			//The ID of the previous menu item is wrong
#define WRONG_NEXT_UCID	4			//The ID of the next menu item is wrong


//extern Prototypes
extern void menu_init(void);
extern void menu_show(void);
/*extern void menu_next(unsigned char step);
extern void menu_prev(unsigned char step);
extern uint8_t ParameterModeActive(void);*/

extern void menu_select(void);

extern void GetSubMenuCount(uint8_t *Size, uint8_t *StartIndex);
void AdjustStartPos(uint8_t *Size, uint8_t *StartIndex);


extern uint16_t GetvalueFromUI(void);
extern void HandOverValueToUI(int16_t value, int16_t upper, int16_t lower, int16_t stepsize);

#endif