/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_PCA10059_
#define _VARIANT_PCA10059_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <nrf.h>

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define NRF_GPIO_PIN_MAP(port, pin)   ((port << 5) | (pin & 0x1F))


/* GPIO */
#define GPIO_PRESENT
#define GPIO_COUNT 2

#define P0_PIN_NUM 32
#define P1_PIN_NUM 16

// Number of pins defined in PinDescription array
#define PINS_COUNT           (47u)
#define NUM_DIGITAL_PINS     (47u)
#define NUM_ANALOG_INPUTS    (4u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
#define PIN_LED1             (6) // P0.06
#define PIN_LED2_R           (8) // P0.08
#define PIN_LED2_G           (41) // P1.09
#define PIN_LED2_B           (12) // P0.12

#define LED_BUILTIN          PIN_LED1

// Buttons
#define PIN_BUTTON           (38) // P1.06

/*
 * Analog pins
 */
#define PIN_A0               (2)  // P0.02
#define PIN_A1               (4)  // P0.04
#define PIN_A2               (29) // P0.29
#define PIN_A3               (31) // P0.31

static const uint8_t A0  = PIN_A0 ;
static const uint8_t A1  = PIN_A1 ;
static const uint8_t A2  = PIN_A2 ;
static const uint8_t A3  = PIN_A3 ;
#define ADC_RESOLUTION    14

// Other pins
#define PIN_NFC1           (9)
#define PIN_NFC2           (10)

/*
 * Serial interfaces
 */
// Serial
#define PIN_SERIAL_RX       (42) // P1.10
#define PIN_SERIAL_TX       (45) // P1.13

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1


#define PIN_SPI_MISO         (13) // P0.13
#define PIN_SPI_MOSI         (15) // P0.15
#define PIN_SPI_SCK          (17) // P0.17

static const uint8_t SS   =   22; // P0.22
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SCL         (29u) // P0.29
#define PIN_WIRE_SDA         (31u) // P0.31

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * Reset Button at P0.18
 */
#define RESET_PIN            18

#ifdef __cplusplus
}
#endif

#endif
