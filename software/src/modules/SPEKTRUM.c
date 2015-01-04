/*
 * SPEKTRUM.c
 *
 * Created: 18.11.2012 01:07:12
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
#include "../config.h"
#include "SPEKTRUM.h"

#include "../FabOS_config.h"
#include "../FabOS/FabOS.h"
#include "cycle_counter.h"

#define delay_us(x) cpu_delay_us(x,BOARD_SYS_HZ)


// input: serial frame
// output: servo signals
int SPEKTRUM_ConvertToServos(int16_t* output, volatile uint8_t* input)
{
	
	/*if(input[0] != 0 || input[1] != 0)
	{
		return 0; // WARNING!! in[0] and in[1] are not always "0" -> Jörgcopter crash.
	}
	*/
	
	// fixme: do some kind of substantial error detection
	
	
	for (uint8_t b = 3; b < SPEKTRUM_FRAME_SIZE; b += 2)
	{
		uint8_t spekChannel = 0x0F & (input[b - 1] >> SPEKTRUM_CHAN_SHIFT);
		
		if (spekChannel < SPEKTRUM_MAX_CHANNEL) 
		{		
			output[spekChannel] = ((long)(input[b - 1] & SPEKTRUM_CHAN_MASK) << 8) + input[b];
		}
	}
	
	return SPEKTRUM_MAX_CHANNEL;
}

int16_t scaleplausServoFromSPEKTRUM(uint16_t raw, int16_t old)
{
	int32_t ret;
	
	#if (SPEKTRUM_BITS == 1024)
	ret = ((int32_t)raw-512)*29*2/5;   // 2048 mode          // 1024 mode
	#endif
	#if (SPEKTRUM_BITS == 2048) 
	ret = ((int32_t)raw-1024)*29/5;   // 2048 mode
	#endif
	
	if(ret >5000 || ret < -5000)
	{
		return old;
	}
	else
	{
		return (int16_t)ret;
	}
}

#define SPEKTRUM_SUPPLY_ON  gpio_clr_gpio_pin(AVR32_PIN_PA30); // switch on 3.3V power for RX
#define SPEKTRUM_SUPPLY_OFF gpio_set_gpio_pin(AVR32_PIN_PA30); // switch off 3.3V power for RX
#define SPEKTRUM_CONF_GPIO gpio_enable_gpio_pin(AVR32_PIN_PB03);gpio_configure_pin(AVR32_PIN_PB03,GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);

#define SPEKTRUM_CONF_SERIAL gpio_configure_pin(AVR32_PIN_PB03,GPIO_DIR_INPUT); gpio_enable_module_pin(AVR32_PIN_PB03, 2 ); // USART1 - RXD "serial input RX")
#define SPEKTRUM_GPIO_HIGH gpio_set_pin_high(AVR32_PIN_PB03);
#define SPEKTRUM_GPIO_LOW  gpio_set_pin_low(AVR32_PIN_PB03);

void SPEKTRUM_Bind(void)
{
	int i = 0;
	
	OS_DISABLEALLINTERRUPTS // to have exact timing
	
	SPEKTRUM_SUPPLY_OFF // switch off 3.3V power for RX
		
	delay_ms(500);
		
	SPEKTRUM_SUPPLY_ON; // switch on 3.3V power for RX
		
	delay_ms(500);
		
	SPEKTRUM_CONF_GPIO // switch RX pin to output

	SPEKTRUM_GPIO_HIGH // set pin high
		
	delay_us(116);

	while(i < SPEKTRUM_BIND_PULSES)
	{
		SPEKTRUM_GPIO_LOW // set pin low
		delay_us(116);
		SPEKTRUM_GPIO_HIGH // set pin high
		delay_us(116);
		i++;
	}

	SPEKTRUM_CONF_SERIAL // switch pin to input
	
	OS_ENABLEALLINTERRUPTS
}


/*

volatile uint8_t spekFrame[SPEK_FRAME_SIZE];

uint16_t readRawRC(uint8_t chan) {
	uint16_t data;

	static uint32_t spekChannelData[SPEK_MAX_CHANNEL];
	if (rcFrameComplete) {
		
		rcFrameComplete = 0;
	}
	#endif
	OS_ENABLEALLINTERRUPTS;// Let's enable the interrupts
	#if defined(SPEKTRUM)
static uint8_t spekRcChannelMap[SPEK_MAX_CHANNEL] = {1,2,3,0,4,5,6};
if (chan >= SPEK_MAX_CHANNEL) {
	data = 1500;
} else {
	
}
#endif
return data; // We return the value correctly copied when the IRQ's where disabled
}
*/


/*

// ***************************************************************
// ** Spektrum Diversity v2.1 - use up to 4 satellite receivers **
// ***************************************************************
// ** Spektrum DSMX Binding Extension by Johannes Stein 2011/03 **
// **                                                           **
// **  3 Pulses = DSM2 1024/22ms                                **
// **  5 Pulses = DSM2 2048/11ms                                **
// **  7 Pulses = DSMX 22ms                                     **
// **  9 Pulses = DSMX 11ms                                     **
// **                                                           **
// ** Target: An Atmel ATtiny2313 (RC-Oscillator @ 8 MHz)       **
// **         controls a 74HC151 Multiplexer                    **
// ***************************************************************
// ** It monitors the data from 4 satellite receivers and       **
// ** connects a valid one to the output via the multiplexer    **
// ***************************************************************
// ** LED-Modes during startup                                  **
// ** ************************                                  **
// ** LED fast flash: Waiting for first signal                  **
// **                                                           **
// ** LED-Modes during operation                                **
// ** **************************                                **
// ** LED flash 1x - 4x: Shows which channel is active every 2s **
// ** LED ON after flash: FAILURE - A used signal was lost      **
// **                                                           **
// ** LED-Modes after Self-Test and pressed button              **
// ** ********************************************              **
// ** LED flashes at 1 Hz: Everything is fine                   **
// ** LED flashes some times: Times flashed -> damaged Channel# **
// ** LED off: check voltage(regulator), button or firmware     **
// ***************************************************************
// ** (c) 2010-0xFFFF Stefan Pendsa                             **
// ** License: don't know - use it and have fun                 **
// ***************************************************************

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define LED_OFF PORTD |= 1 << PD0
#define LED_ON  PORTD &= ~(1 << PD0)

volatile unsigned char Timer0Event = 0;
volatile unsigned int Timer1Event;
volatile unsigned char BlinkCode = 0;
volatile unsigned char failure = 0;

unsigned char eedummy EEMEM;                                                            // Dummy-Variable for Address 0 in EEPROM
unsigned char eecheck EEMEM;                                                            // Check-Variable
unsigned char bind;                                                                                     // Bind-Pulse-Counter (in RAM)
unsigned char eebind EEMEM;                                                                     // Bind-Pulse-Counter (in EEPROM)



ISR(TIMER0_OVF_vect)                                                                            // Triggered every 32,768 msec
{
        if (Timer0Event > 64)                                                                   // Max. 64*32 msec = 2,097152 sec
        {
                Timer0Event = 0;
                LED_OFF;
        }
       
        if(Timer0Event == 0 && BlinkCode > 0) LED_ON;
        else if(Timer0Event == 4 && BlinkCode > 0) LED_OFF;
        else if(Timer0Event == 8 && BlinkCode > 1) LED_ON;
        else if(Timer0Event == 12 && BlinkCode > 1) LED_OFF;
        else if(Timer0Event == 16 && BlinkCode > 2) LED_ON;
        else if(Timer0Event == 20 && BlinkCode > 2) LED_OFF;
        else if(Timer0Event == 24 && BlinkCode > 3) LED_ON;
        else if(Timer0Event == 28 && BlinkCode > 3) LED_OFF;
        else if(Timer0Event == 35 && failure == 1) LED_ON;
        else if(Timer0Event == 55 && failure == 1) LED_OFF;

        Timer0Event++;
}


ISR(TIMER1_OVF_vect)                                                                            // Triggered every 8,192 msec
{
        Timer1Event++;
}


void SelectSat(unsigned char sat)
{
        PORTD = (PORTD & 0b1100111) | (sat << 3);                               // Select the input for 74HC151
        _delay_us(10);                                                                                  // Wait for stable state
}



void ResetTimer1(void)
{
        TCNT1H = 0;
        TCNT1L = 0;
        Timer1Event = 0;
}



void Binding(void)                                                                                      // Binding sequence (for Spektrum sats only)
{
        unsigned char i = 0;

        DDRB = 0b11110000;                                                                              // Sat-Pins to output
        _delay_ms(80);                                                                                  // Let them time to boot up
       
        for (i=0;i<bind;i++)                                                                    // Send negative 100µs pulses to all sat's
        {
                PORTB = 0b00000000;
                _delay_us(100);
                PORTB = 0b11110000;
                _delay_us(100);
        }

        for (i=0;i<bind;i++)                                                                    // Flash the number of used pulses to the LED
        {
                LED_ON;
                _delay_ms(100);
                LED_OFF;
                _delay_ms(250);
        }

        DDRB = 0b00000000;                                                                              // Sat-Pins to input again
       
        bind += 2;
        if (bind > 9) bind = 3;                                                                 // 3, 5, 7, 9 pulses, then start with 3 again
        eeprom_write_byte(&eebind, bind);                                               // Save increased bind-pulse-counter for next binding.
        _delay_ms(500);
}



void Testing(void)                                                                                      // Self Test
{
        unsigned char error = 0;
        unsigned char i = 0;

        DDRB = 0b11110000;                                                                              // Port B Output for satellites, Input for feedback

        PORTB = 0b01010000;                                                                             // Test Pattern
        SelectSat(0);
        if (!(PINB & (1<<PB3))) error = 1;
        SelectSat(1);
        if (PINB & (1<<PB3)) error = 2;
        SelectSat(2);
        if (!(PINB & (1<<PB3))) error = 3;
        SelectSat(3);
        if (PINB & (1<<PB3)) error = 4;

        PORTB = 0b10100000;                                                                             // Another (inverted) Test Pattern
        SelectSat(0);
        if (PINB & (1<<PB3)) error = 1;
        SelectSat(1);
        if (!(PINB & (1<<PB3))) error = 2;
        SelectSat(2);
        if (PINB & (1<<PB3)) error = 3;
        SelectSat(3);
        if (!(PINB & (1<<PB3))) error = 4;

        DDRB = 0b00000000;                                                                              // Port B Input again

        while (PIND & (1<<PD6));                                                                // Wait for Bind-Switch

        while(1)                                                                                                // Never return
        {
                if (error == 0)                                                                         // When no error occured -> LED flashes at 1 Hz
                {
                        LED_ON;
                        _delay_ms(100);
                        LED_OFF;
                        _delay_ms(900);
                }
                else
                {
                        for (i=0;i<error;i++)                                                   // When error occured -> Flash-Out the Errorcode
                        {
                                LED_ON;
                                _delay_ms(100);
                                LED_OFF;
                                _delay_ms(400);
                        }
                        _delay_ms(1000);
                }

        }

}



int main(void)
{
        unsigned char i = 0;
        unsigned char active[4];
        unsigned char active_lo[4];
        unsigned char active_hi[4];
        unsigned char sat = 99;


        DDRB = 0b00000000;                                                                              // Port B Input for satellites and feedback
        DDRD = 0b0011001;                                                                               // Port D Output for MUX and LED, Input for Switch & Test
        PORTB = 0b11110000;                                                                             // Port B Pullup's for (unused) satellites
        PORTD = 0b1100001;                                                                              // Port D Pullup's for Switch & Test, LED off

    if (eeprom_read_byte(&eecheck) != 0x84)                                     // Invalid Data in EEPROM -> initialize
        {
                eeprom_write_byte(&eecheck, 0x84);
                bind = 3;
                eeprom_write_byte(&eebind, bind);
        }
        else bind = eeprom_read_byte(&eebind);                                  // Valid Data in EEPROM -> read bind-pulse-counter


        if (!(PIND & (1<<PD5))) Testing();                                              // Initiate Self-Test when Test-Pad is low
        if (!(PIND & (1<<PD6))) Binding();                                              // Initiate Binding when Bind-Button is pressed


        for (i=0;i<4;i++)                                                                               // Reset active-arrays
        {
                active[i] = 0;
                active_lo[i] = 0;
                active_hi[i] = 0;
        }

        _delay_ms(100);

        TCCR0B = ( 1 << CS00 ) | ( 1 << CS02 );                                 // Timer0 Prescaler = 1024 -> 32,768 msec
        TCCR1B = ( 1 << CS10 );                                                                 // Timer1 Prescaler = 1 -> 8,192 msec
        TIMSK = ( 1 << TOIE0 ) | ( 1 << TOIE1 );                                // Timer0+1 Overflow Interrupt Enable
        sei();                                                                                                  // Global Interrupts enable

        ResetTimer1();
        while(sat == 99)                                                                                // Wait for first signal
        {
                if (Timer1Event == 10) LED_ON;                                          // Blink while waiting
                if (Timer1Event == 20)
                {
                        LED_OFF;
                        Timer1Event = 0;
                }

                while(Timer1Event < 3)                                                          // Check active satellites (for 3*8=24ms)
                {
                        for (i=0;i<4;i++)
                        {
                                if (PINB & (1<<(i+4))) active_hi[i] = 1;
                                else active_lo[i] = 1;
                        }
                }

                for (i=0;i<4;i++)                                                                       // When an input had low AND high signals, mark it as active
                {
                        if (active_lo[i] == 1 && active_hi[i] == 1) active[i] = 1;
                }


                for (i=0;i<4;i++)                                                                       // Select first active satellite
                {
                        if (active[i] == 1)
                        {
                                SelectSat(i);
                                sat = i;
                                BlinkCode = i+1;
                                break;
                        }
                }

        }




        while(1)                                                                                                // Main-Loop
        {
                for (i=0;i<4;i++)                                                                       // Reset active-arrays
                {
                        active[i] = 0;
                        active_lo[i] = 0;
                        active_hi[i] = 0;
                }

                ResetTimer1();
                while(Timer1Event < 3)                                                          // Check active satellites (for 3*8=24ms)
                {
                        for (i=0;i<4;i++)
                        {
                                if (PINB & (1<<(i+4))) active_hi[i] = 1;
                                else active_lo[i] = 1;
                        }
                }


                for (i=0;i<4;i++)                                                                       // When an input had low AND high signals, mark it as active
                {
                        if (active_lo[i] == 1 && active_hi[i] == 1) active[i] = 1;
                }
               

                if (active[0] == 0 && active[1] == 0 && active[2] == 0 && active[3] == 0 && sat != 99)
                {
                        failure = 1;                                                                    // Set Failure-LED when the signal is lost completely
                        BlinkCode = 0;
                        sat = 99;
                }
               

                for (i=0;i<4;i++)                                                                       // Select active satellite (priorized)
                {
                        if (active[i] == 1)
                        {
                                SelectSat(i);
                                if (sat != i) failure = 1;                                      // Set Failure-LED when the active satellite changes
                                sat = i;
                                BlinkCode = i+1;
                                break;
                        }
                }



                if (!(PIND & (1<<PD6))) failure = 0;                            // Reset Failure-LED when Bind-Button is pressed
        }


*/

