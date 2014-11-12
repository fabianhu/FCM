/*
 * eic.c
 * Driver for the AVR32 UC3B external interrupt controller, as ASF is way too complicated.
 *
 * Created: 27.11.2012 23:29:05
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

#include <asf.h>
#include "..\FabOS_config.h"
#include "..\FabOS\FabOS.h"
#include "eic.h"

// enable and activate EIC pin
void eic_enable_channel(uint32_t channel)
{
	uint32_t mask = 1<<channel;
	
	AVR32_EIC.ier = mask;		// enable interrupt propagation to INTC.
	
	AVR32_EIC.mode &= ~mask;	//mode edge = 0
	
	AVR32_EIC.edge |= mask;		//edge rising = 1
	
	//level don't care

	AVR32_EIC.filter |= mask;	//filter on
	
	//sync = 0 (all & default)

	AVR32_EIC.en = mask;	// enable interrupt generation

}

void eic_disable_channel(uint32_t channel)
{
	OS_DISABLEALLINTERRUPTS;
	
	uint32_t mask = 1<<channel;
	
	AVR32_EIC.idr = mask;		// enable interrupt propagation to INTC.
	
	AVR32_EIC.mode &= ~mask;	//mode edge = 0
	
	AVR32_EIC.edge |= mask;		//edge rising = 1
	
	//level don't care

	AVR32_EIC.filter |= mask;	//filter on
	
	//sync = 0 (all & default)

	AVR32_EIC.dis = mask;	// enable interrupt generation
	
	eic_clear_int(channel); // just in case...
	
	OS_ENABLEALLINTERRUPTS;

}

void eic_sw_edge_rising(uint32_t pin)
{
	AVR32_EIC.edge |= (1 << pin);
}

void eic_sw_edge_falling(uint32_t pin)
{
	AVR32_EIC.edge &= ~(1 << pin);
}

void eic_clear_int(uint32_t pin)
{
	irqflags_t flags;
	flags = cpu_irq_save(); // store interrupt state and disable interrupts (todo try without)
	AVR32_EIC.icr = 1 << pin;
	//eic->isr;
	cpu_irq_restore(flags); // restore interrupt status
}

int32_t eic_is_rising(uint32_t pin)
{
	return AVR32_EIC.edge & (1 << pin);
}