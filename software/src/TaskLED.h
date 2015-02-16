/*
 * TaskLED.h
 *
 * Created: 02.11.2012 22:20:27
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


#ifndef TASKLED_H_
#define TASKLED_H_

#define LED_RED_TOG		gpio_tgl_gpio_pin(AVR32_PIN_PA20) // LED Toggle
#define LED_RED_ON		gpio_set_gpio_pin(AVR32_PIN_PA20) // LED ON
#define LED_RED_OFF		gpio_clr_gpio_pin(AVR32_PIN_PA20) // LED Off
#define LED_GREEN_TOG	gpio_tgl_gpio_pin(AVR32_PIN_PA21) // LED Toggle
#define LED_GREEN_ON	gpio_set_gpio_pin(AVR32_PIN_PA21) // LED ON
#define LED_GREEN_OFF	gpio_clr_gpio_pin(AVR32_PIN_PA21) // LED Off
#define LED_BLUE_TOG	gpio_tgl_gpio_pin(AVR32_PIN_PA22) // LED Toggle
#define LED_BLUE_ON		gpio_set_gpio_pin(AVR32_PIN_PA22) // LED ON
#define LED_BLUE_OFF	gpio_clr_gpio_pin(AVR32_PIN_PA22) // LED Off

FlightState_t LED_GetFlightstate(void);
void LED_SetFlightstate(FlightState_t fs);
uint32_t LED_GetLastFlightstateChanged(void);

#endif /* TASKLED_H_ */