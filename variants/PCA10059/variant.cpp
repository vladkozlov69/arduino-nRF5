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

#include "variant.h"

const PinDescription g_ADigitalPinMap[] = {
  {0,0},  // D0
  {0,1},  // D1
  {0,2},
  {0,3},
  {0,4},
  {0,5},
  {0,6},
  {0,7},
  {0,8},
  {0,9},
  {0,10},
  {0,11},
  {0,12},
  {0,13},
  {0,14},
  {0,15},
  {0,16},
  {0,17},
  {0,18},
  {0,19},
  {0,20},
  {0,21},
  {0,22},
  {0,23},
  {0,24},
  {0,25},
  {0,26},
  {0,27},
  {0,28},
  {0,29},
  {0,30},
  {0,31},
  {1,0},  // D32
  {1,1},  // D1
  {1,2},
  {1,3},
  {1,4},
  {1,5},
  {1,6},  // D38
  {1,7},
  {1,8},
  {1,9},  // D41
  {1,10}, // D42
  {1,11},
  {1,12},
  {1,13}, // D45
  {1,14},
  {1,15}, // D47
};

/**
 * Function for configuring UICR_REGOUT0 register
 * to set GPIO output voltage to 3.0V.
 */
static void gpio_output_voltage_setup(void)
{
    // Configure UICR_REGOUT0 register only if it is set to default value.
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
        (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_3V0 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        // System reset is needed to update UICR registers.
        NVIC_SystemReset();
    }
}

void initVariant()
{
  // If nRF52 USB Dongle is powered from USB (high voltage mode),
  // GPIO output voltage is set to 1.8 V by default, which is not
  // enough to turn on green and blue LEDs. Therefore, GPIO voltage
  // needs to be increased to 3.0 V by configuring the UICR register.

  if (NRF_POWER->MAINREGSTATUS &
     (POWER_MAINREGSTATUS_MAINREGSTATUS_High << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos))
  {
      gpio_output_voltage_setup();
  }
}