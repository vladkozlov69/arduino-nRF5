/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
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

#include "nrf.h"

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

void pinMode( uint32_t ulPin, uint32_t ulMode )
{
  if (ulPin >= PINS_COUNT) {
    return;
  }

  #ifdef NRF52840
    ulPin = NRF_GPIO_PIN_MAP(g_ADigitalPinMap[ulPin].ulPort,g_ADigitalPinMap[ulPin].ulPin);
  #else
    ulPin = g_ADigitalPinMap[ulPin];
  #endif
  NRF_GPIO_Type * p_reg = nrf_gpio_pin_port_decode(&ulPin);

  // Set pin mode according to chapter '22.6.3 I/O Pin Configuration'
  switch ( ulMode )
  {
    case INPUT:
      // Set pin to input mode
      p_reg->PIN_CNF[ulPin] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
    break ;

    case INPUT_PULLUP:
      // Set pin to input mode with pull-up resistor enabled
      p_reg->PIN_CNF[ulPin] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Pullup      << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
    break ;

    case INPUT_PULLDOWN:
      // Set pin to input mode with pull-down resistor enabled
      p_reg->PIN_CNF[ulPin] = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Pulldown    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
    break ;

    case OUTPUT:
      // Set pin to output mode
      p_reg->PIN_CNF[ulPin] = ((uint32_t)GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
    break ;

    case OUTPUT_OPEN_DRAIN:
      // Set pin to output mode open drain 5mA max sink!
      p_reg->PIN_CNF[ulPin] = ((uint32_t)GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_H0D1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
    break ;

    default:
      // do nothing
    break ;
  }
}

void digitalWrite( uint32_t ulPin, uint32_t ulVal )
{
  if (ulPin >= PINS_COUNT) {
    return;
  }

  #ifdef NRF52840
    ulPin = NRF_GPIO_PIN_MAP(g_ADigitalPinMap[ulPin].ulPort,g_ADigitalPinMap[ulPin].ulPin);
  #else
    ulPin = g_ADigitalPinMap[ulPin];
  #endif
  NRF_GPIO_Type * p_reg = nrf_gpio_pin_port_decode(&ulPin);

  switch ( ulVal )
  {
    case LOW:
      p_reg->OUTCLR = (1UL << ulPin);
    break ;

    default:
      p_reg->OUTSET = (1UL << ulPin);
    break ;
  }

  return ;
}

int digitalRead( uint32_t ulPin )
{
  if (ulPin >= PINS_COUNT) {
    return 0;
  }
  #ifdef NRF52840
    ulPin = NRF_GPIO_PIN_MAP(g_ADigitalPinMap[ulPin].ulPort,g_ADigitalPinMap[ulPin].ulPin);
  #else
    ulPin = g_ADigitalPinMap[ulPin];
  #endif
  NRF_GPIO_Type * p_reg = nrf_gpio_pin_port_decode(&ulPin);

  return ((p_reg->IN >> ulPin) & 1UL) ? HIGH : LOW ;
}
/**
 * @brief Function for extracting port and relative pin number from absolute pin number.
 *
 * @param[inout] Pointer to absolute pin number which is overriden by relative to port pin number.
 *
 * @return Pointer to port register set.
 *
 */
NRF_GPIO_Type * nrf_gpio_pin_port_decode(uint32_t * p_pin)
{
#if (GPIO_COUNT == 1)
    // The oldest definition case
    return NRF_GPIO;
#else
    if (*p_pin < P0_PIN_NUM)
    {
        return NRF_P0;
    }
    else
    {
        *p_pin = *p_pin & (P0_PIN_NUM - 1);
        return NRF_P1;
    }
#endif
}


#ifdef __cplusplus
}
#endif
