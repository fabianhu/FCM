/*
 * board.c
 *
 * Created: 04.10.2012 21:06:22
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

#include <conf_board.h>

void init_pins(void);
void init_xtal_pll(void);


void init_pins(void)
{
	// init all pins


	// switch the GPIOs to their functions:
	// 0 = A, 1 = B, 2 = C, (3 = D only for UC3B512)
	static const gpio_map_t iomap =
	{
		{AVR32_PIN_PA05, 0 }, // EIC - EXTINT[0] "Int1 Accel"
		{AVR32_PIN_PA06, 0 }, // EIC - EXTINT[1] "RC2"

		{AVR32_PIN_PA09, 0 }, // TWI - SCL
		{AVR32_PIN_PA10, 0 }, // TWI - SDA
		{AVR32_PIN_PA11, 2 }, // PWM - PWM[0]  "PWM 1"
		{AVR32_PIN_PA12, 2 }, // PWM - PWM[1]  "PWM 2"
		{AVR32_PIN_PA13, 1 }, // PWM - PWM[2]  "PWM 3"
		{AVR32_PIN_PA14, 2 }, // EIC - EXTINT[2] "RC3" 
		{AVR32_PIN_PA15, 0 }, // SPI0 - SCK
		{AVR32_PIN_PA16, 2 }, // PWM - PWM[4]  "PWM 5"

		{AVR32_PIN_PA23, 2 }, // EIC - EXTINT[3] "RC4" 
		{AVR32_PIN_PA24, 1 }, // SPI0 - NPCS[0]
		{AVR32_PIN_PA25, 1 }, // PWM - PWM[3]  "PWM 4"
		{AVR32_PIN_PA26, 1 }, // USART2 - TXD
		{AVR32_PIN_PA27, 1 }, // USART2 - RXD
		{AVR32_PIN_PA28, 2 }, // SPI0 - MISO
		{AVR32_PIN_PA29, 2 }, // SPI0 - MOSI

		{AVR32_PIN_PA31, 2 }, // PWM - PWM[6]  "PWM 6"
		{AVR32_PIN_PB00, 0 }, // TC - A0
		{AVR32_PIN_PB01, 0 }, // TC - B0
		{AVR32_PIN_PB02, 0 }, // EIC - EXTINT[6] "Gyro L3GD20 Int2"
			
		// next will be switched dynamically, if serial is required.
		{AVR32_PIN_PB03, 0 }, // EIC - EXTINT[7] "RC1"
		//{AVR32_PIN_PB03, 2 }, // USART1 - RXD "serial input RX"



		{AVR32_PIN_PB10, 2 }, // USART0 - RXD
		{AVR32_PIN_PB11, 2 } // USART0 - TXD
	};
	// enable modules
	gpio_enable_module(iomap, sizeof(iomap) / sizeof(iomap[0]));


// ***************************************
// init GPIOs

gpio_enable_gpio_pin(AVR32_PIN_PA20); // LED
gpio_enable_gpio_pin(AVR32_PIN_PA21); // LED
gpio_enable_gpio_pin(AVR32_PIN_PA22); // LED
gpio_enable_gpio_pin(AVR32_PIN_PA30); // RX power switch 3.3V

/*
// debug 
gpio_enable_gpio_pin(AVR32_PIN_PA06);
gpio_enable_gpio_pin(AVR32_PIN_PA14);
gpio_enable_gpio_pin(AVR32_PIN_PA23);
gpio_configure_pin(AVR32_PIN_PA06, GPIO_DIR_OUTPUT | GPIO_DRIVE_HIGH);
gpio_configure_pin(AVR32_PIN_PA14, GPIO_DIR_OUTPUT | GPIO_DRIVE_HIGH);
gpio_configure_pin(AVR32_PIN_PA23, GPIO_DIR_OUTPUT | GPIO_DRIVE_HIGH);
*/

gpio_configure_pin(AVR32_PIN_PA20, GPIO_DIR_OUTPUT | GPIO_DRIVE_HIGH);
gpio_configure_pin(AVR32_PIN_PA21, GPIO_DIR_OUTPUT | GPIO_DRIVE_HIGH);
gpio_configure_pin(AVR32_PIN_PA22, GPIO_DIR_OUTPUT | GPIO_DRIVE_HIGH);
gpio_configure_pin(AVR32_PIN_PA30, GPIO_DIR_OUTPUT | GPIO_DRIVE_HIGH);

// enable pull-ups at Interrupt-input-pins and RX
gpio_enable_pin_pull_up(AVR32_PIN_PA06); // RC2
gpio_enable_pin_pull_up(AVR32_PIN_PA14); // RC3
gpio_enable_pin_pull_up(AVR32_PIN_PA23); // RC4

gpio_enable_pin_pull_up(AVR32_PIN_PB10); // USART0
gpio_enable_pin_pull_up(AVR32_PIN_PB03); // USART1
gpio_enable_pin_pull_up(AVR32_PIN_PA27); // USART2

gpio_enable_pin_pull_up(AVR32_PIN_PB02); // EIC - EXTINT[6] "Gyro L3GD20 Int2"
gpio_enable_pin_pull_up(AVR32_PIN_PA05); // EIC - EXTINT[0] "Int1 Accel"

}

void init_xtal_pll(void)
{
	// spin up revolutions
	
	pm_freq_param_t p;
	p.cpu_f = BOARD_SYS_HZ;
	p.osc0_f = BOARD_OSC0_HZ;
	p.osc0_startup = AVR32_PM_OSCCTRL0_STARTUP_16384_RCOSC;
	p.pba_f = BOARD_SYS_HZ;
	pm_configure_clocks(&p);
	// now we run with 48MHz <---- true! // 60 MHz does not work...
}