/*
 * servo_out.c
 *
 * Created: 06.01.2012 14:17:58
 *
 * (c) 2012-2015 by Fabian Huslik
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
#include "servo_out.h"
#include "servo_in.h"
#include "FabOS_config.h"
#include "FabOS/FabOS.h"
#include "vector.h"
#include "governing.h"
#include "menu/menu.h"
#include "menu/menu_variant.h"
#include "config.h"

#define SERVO_DUTY1ms	BOARD_SYS_HZ / 2 /50 / 20
#define SERVO_PERIOD	BOARD_SYS_HZ / 2 / 50 /2 // now with 100Hz

#define SERVO_SCALE 10000L // max value for servo out

static void PWM_setChannel( uint32_t duty, uint8_t n );

__attribute__((__interrupt__))
static void isr_servo_out(void)
{
	// clear int request
	AVR32_PWM.isr; // dummy read for clearing the interrupt
	
	// Perform governor
	OS_SetEventFromISR(OSTSK_CONTROL,OSEVT_CTLWAKEUP);
}



void servo_out_init(void)
{
	// for this time, we assume, that the PWM_CLK is running with 48MHz as well.

	// init PWM
	pwm_opt_t pwmopt;
	pwmopt.prea = 1; // = PWM_CLK / 2	
	pwmopt.diva = 1; // = 1
	pwmopt.preb = 0; // off
	pwmopt.divb = 0; // off
	pwm_init(&pwmopt);
	
	avr32_pwm_cmr_t pwmcmr;
	pwmcmr.calg = 0; // left aligned
	pwmcmr.cpd = 0;  // 1 = period 0 = duty (not relevant for init.)
	pwmcmr.cpol = 1; // waveform starts with high level
	pwmcmr.cpre = 1; // CLK_PWM/2
	
	avr32_pwm_channel_t pwmch;
	// pwmch.ccnt = 0; // read only member! (const)
	pwmch.cdty = SERVO_DUTY1ms; // duty cycle // 1 ms as a default
	pwmch.CMR = pwmcmr; // Channel mode register, see above.
	pwmch.cprd = SERVO_PERIOD; 
	pwmch.cupd = 0; // update register, 

	pwm_channel_init(AVR32_PWM_CHID0,&pwmch);
	pwm_channel_init(AVR32_PWM_CHID1,&pwmch);
	pwm_channel_init(AVR32_PWM_CHID2,&pwmch);
	pwm_channel_init(AVR32_PWM_CHID3,&pwmch);
	pwm_channel_init(AVR32_PWM_CHID4,&pwmch);
	pwm_channel_init(AVR32_PWM_CHID6,&pwmch); // !! ch 5 is skipped in HW FCM 2.1 !!
	
	
	// int pwm_sync_update_channel 	(	unsigned int 	channel_id,	const avr32_pwm_channel_t * 	pwm_channel )
	
	pwm_start_channels 	(	0b1011111	); // !! ch 5 is skipped in HW FCM 2.1 !!
	
	
	INTC_register_interrupt(&isr_servo_out, AVR32_PWM_IRQ, AVR32_INTC_INTLEVEL_INT1); 
	
	AVR32_PWM.IER.chid0 = 1;
	

}

void PWM_setChannel( uint32_t duty, uint8_t n ) 
{
		avr32_pwm_cmr_t pwmcmr;
	pwmcmr.calg = 0; // left aligned
	pwmcmr.cpd  = 0; // 0 = update duty (very relevant here!)
	pwmcmr.cpol = 1; // waveform starts with high level
	pwmcmr.cpre = 1; // CLK_PWM/2
	
	avr32_pwm_channel_t pwmch;
	// pwmch.ccnt = 0; // read only member! (const)
	pwmch.cdty = duty;
	pwmch.CMR  = pwmcmr; // Channel mode register, see above.
	pwmch.cprd = SERVO_PERIOD; 
	pwmch.cupd = duty; // update register, 
	
	if(n == 5)
		n = 6; // !! ch 5 is skipped in HW FCM 2.1 !!

	pwm_async_update_channel(n,&pwmch);
}


void servo_out_set(uint8_t n, uint32_t val) // defined as 0..SERVO_SCALE
{
	uint32_t duty;
	
	if( val > SERVO_SCALE) return;
	
	
	duty = SERVO_DUTY1ms + (val*(SERVO_DUTY1ms))/SERVO_SCALE;
	
	PWM_setChannel(duty, n);
}

void servo_out_zero(void)
{
	PWM_setChannel(0,0);
	PWM_setChannel(1,0);
	PWM_setChannel(2,0);
	PWM_setChannel(3,0);
	PWM_setChannel(4,0);
	PWM_setChannel(6,0);
}


// motor configuration sensitivity map.
/* scaling map 0..100%
	sum of absolutes for all motors (lines) should be equal for each row.
	sum for all axes(x,y,z) must equal 0;
	x: factor of motor, in which it contributes to the x-axis
	y: factor of motor, in which it contributes to the y-axis
	z: factor of motor, in which it contributes to the z-axis
	a: sensitivity in which extent the motor contributes to the total lift

*/
#ifdef QUADX
const motor_t mot_conf[NUMOUTPUTS] = {
//		x		y		z		a		// commands
	{	25	, 	25	, 	25	, 	100	},	// motor 1
	{	25	, 	-25	, 	-25	, 	100	},	// motor 2
	{	-25	, 	-25	, 	25	, 	100	},	// motor 3
	{	-25	, 	25	, 	-25	, 	100	},	// motor 4
	{	0	, 	0	, 	0	, 	100	},	// motor 5
	{	0	, 	0	, 	0	, 	100	}	// motor 6
};
	#warning "Quad-X configured."
	
#elif defined (ULICOPTER_QUAD)
const motor_t mot_conf[NUMOUTPUTS] = {
//		x		y		z		a		// commands
	{	28	,	22	, 	-25	, 	100	},	// motor 1
	{	28	,	-22	, 	25	, 	100	},	// motor 2
	{	-28	,	-22	, 	-25	, 	100	},	// motor 3
	{	-28	,	22	, 	25	, 	100	},	// motor 4
	{	0	,	0	, 	0	, 	100	},	// motor 5
	{	0	,	0	, 	0	, 	100	}	// motor 6
};
	#warning "UliCopter_Quad configured."
	
#elif defined (HEXX)
const motor_t mot_conf[NUMOUTPUTS] = {
//		x		y		z		a		// commands
	{	25	, 	12	, 	16	, 	100	},	// motor 1
	{	25	, 	-12	, 	-16	, 	100	},	// motor 2
	{	0	, 	-25	, 	16	, 	100	},	// motor 3
	{	-25	, 	-12	, 	-16	, 	100	},	// motor 4
	{	-25	, 	12	, 	16	, 	100	},	// motor 5
	{	0	, 	25	, 	-16	, 	100	}	// motor 6
};
	#warning "Hexa-X configured."
	
#elif defined (PLANE)
const motor_t mot_conf[NUMOUTPUTS] = {
//		x		y		z		a		// commands
	{	0	, 	0	, 	0	, 	100	},	// motor 1
	{	0	, 	100	, 	0	, 	0	},	// motor 2
	{	100	, 	0	, 	0	, 	0	},	// motor 3
	{	0	, 	0	, 	100	, 	0	},	// motor 4
	{	0	, 	0	, 	0	, 	0	},	// motor 5
	{	0	, 	0	, 	0	, 	0	}	// motor 6
};
#warning "Plane configured."

#elif defined (ULICOPTER_HEX)
const motor_t mot_conf[NUMOUTPUTS] = {
//		x		y		z		a		// commands
	{	25	,	22	, 	-12	, 	100	},	// motor 1
	{	25	,	-22	, 	12	, 	100	},	// motor 2
	{	0	,	-17	, 	-25	, 	100	},	// motor 3
	{	-25	,	-12	, 	12	, 	100	},	// motor 4
	{	-25	,	12	, 	-12	, 	100	},	// motor 5
	{	0	,	17	, 	25	, 	100	}	// motor 6
};
#warning "UliCopter_Hex configured."

#else

#error "Configure copter type!"

#endif

	int32_t mix_out[6];

void MixSetOut(int32_t ox, int32_t oy, int32_t oz, int32_t oa)
{

	int32_t temp;
	
	// PWM set
	
	for(int32_t i=0; i<NUMOUTPUTS; i++)
	{
		// directional change component
		temp =	mot_conf[i].sens_x * ox + 
				mot_conf[i].sens_y * oy +
				mot_conf[i].sens_z * oz ;

		// total thrust component
		mix_out[i] = mot_conf[i].sens_a * oa + temp;

		// set PWM at output
		servo_out_set(i,limit(mix_out[i]/100L,myPar.idle_power.sValue,10000)); // limit to max range. not narrower.
	}
	
}


void DOPgmESCs(void)
{
	int32_t cmd_thr, cmd_roll, cmd_elev, cmd_yaw;

	servo_in_getCmd(&cmd_thr, &cmd_roll, &cmd_elev, &cmd_yaw); // +-4000 for rot; 1000..8000 for throttle

	
	servo_out_set(0,cmd_thr);
	servo_out_set(1,cmd_thr);
	servo_out_set(2,cmd_thr);
	servo_out_set(3,cmd_thr);
	servo_out_set(4,cmd_thr);
	servo_out_set(5,cmd_thr);
	
}
