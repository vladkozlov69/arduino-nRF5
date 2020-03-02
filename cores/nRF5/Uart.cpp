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

#include "Uart.h"
#include "Arduino.h"
#include "wiring_private.h"

Uart::Uart(NRF_UART_Type *_nrfUart, IRQn_Type _IRQn, uint8_t _pinRX, uint8_t _pinTX)
{
  nrfUart = _nrfUart;
  IRQn = _IRQn;
  uc_pinRX = _pinRX;
  uc_pinTX = _pinTX;
  pinMode(uc_pinRX, INPUT);
  pinMode(uc_pinTX, OUTPUT);
  digitalWrite(uc_pinTX, HIGH);
  uc_hwFlow = 0;
}

Uart::Uart(NRF_UART_Type *_nrfUart, IRQn_Type _IRQn, uint8_t _pinRX, uint8_t _pinTX, uint8_t _pinCTS, uint8_t _pinRTS)
{
  nrfUart = _nrfUart;
  IRQn = _IRQn;
  uc_pinRX = _pinRX;
  uc_pinTX = _pinTX;
  uc_pinCTS = _pinCTS;
  uc_pinRTS = _pinRTS;
  pinMode(uc_pinRX, INPUT);
  pinMode(uc_pinCTS, INPUT);
  pinMode(uc_pinTX, OUTPUT);
  pinMode(uc_pinRTS, OUTPUT);
  digitalWrite(uc_pinTX, HIGH);
  digitalWrite(uc_pinRTS, HIGH);
  uc_hwFlow = 1;
}

#ifdef ARDUINO_GENERIC
void Uart::setPins(uint8_t _pinRX, uint8_t _pinTX)
{
  uc_pinRX = _pinRX;
  uc_pinTX = _pinTX;
}

void Uart::setPins(uint8_t _pinRX, uint8_t _pinTX, uint8_t _pinCTS, uint8_t _pinRTS)
{
  uc_pinRX = _pinRX;
  uc_pinTX = _pinTX;
  uc_pinCTS = _pinCTS;
  uc_pinRTS = _pinRTS;
}
#endif // ARDUINO_GENERIC

void Uart::begin(unsigned long baudrate)
{
  begin(baudrate, (uint8_t)SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint16_t /*config*/)
{
  #ifdef NRF52840
  nrfUart->PSELTXD = NRF_GPIO_PIN_MAP(g_ADigitalPinMap[uc_pinTX].ulPort,g_ADigitalPinMap[uc_pinTX].ulPin);
  nrfUart->PSELRXD = NRF_GPIO_PIN_MAP(g_ADigitalPinMap[uc_pinRX].ulPort,g_ADigitalPinMap[uc_pinRX].ulPin);
  #else
  nrfUart->PSELTXD = g_ADigitalPinMap[uc_pinTX];
  nrfUart->PSELRXD = g_ADigitalPinMap[uc_pinRX];
  #endif

  if (uc_hwFlow == 1) {
    #ifdef NRF52840
    nrfUart->PSELCTS = NRF_GPIO_PIN_MAP(g_ADigitalPinMap[uc_pinCTS].ulPort,g_ADigitalPinMap[uc_pinCTS].ulPin);
    nrfUart->PSELRTS = NRF_GPIO_PIN_MAP(g_ADigitalPinMap[uc_pinRTS].ulPort,g_ADigitalPinMap[uc_pinRTS].ulPin);
    #else
    nrfUart->PSELCTS = g_ADigitalPinMap[uc_pinCTS];
    nrfUart->PSELRTS = g_ADigitalPinMap[uc_pinRTS];
    #endif
    nrfUart->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) | UART_CONFIG_HWFC_Enabled;
  } else {
    nrfUart->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) | UART_CONFIG_HWFC_Disabled;
  }

  uint32_t nrfBaudRate;

#ifdef NRF52
  if (baudrate <= 1200) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud1200;
  } else if (baudrate <= 2400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud2400;
  } else if (baudrate <= 4800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud4800;
  } else if (baudrate <= 9600) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud9600;
  } else if (baudrate <= 14400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud14400;
  } else if (baudrate <= 19200) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud19200;
  } else if (baudrate <= 28800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud28800;
  } else if (baudrate <= 38400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud38400;
  } else if (baudrate <= 57600) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud57600;
  } else if (baudrate <= 76800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud76800;
  } else if (baudrate <= 115200) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud115200;
  } else if (baudrate <= 230400) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud230400;
  } else if (baudrate <= 250000) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud250000;
  } else if (baudrate <= 460800) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud460800;
  } else if (baudrate <= 921600) {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud921600;
  } else {
    nrfBaudRate = UARTE_BAUDRATE_BAUDRATE_Baud1M;
  }
#else
  if (baudrate <= 1200) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud1200;
  } else if (baudrate <= 2400) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud2400;
  } else if (baudrate <= 4800) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud4800;
  } else if (baudrate <= 9600) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud9600;
  } else if (baudrate <= 14400) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud14400;
  } else if (baudrate <= 19200) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud19200;
  } else if (baudrate <= 28800) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud28800;
  } else if (baudrate <= 38400) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud38400;
  } else if (baudrate <= 57600) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud57600;
  } else if (baudrate <= 76800) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud76800;
  } else if (baudrate <= 115200) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud115200;
  } else if (baudrate <= 230400) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud230400;
  } else if (baudrate <= 250000) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud250000;
  } else if (baudrate <= 460800) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud460800;
  } else if (baudrate <= 921600) {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud921600;
  } else {
    nrfBaudRate = UART_BAUDRATE_BAUDRATE_Baud1M;
  }
#endif

  nrfUart->BAUDRATE = nrfBaudRate;

  nrfUart->ENABLE = UART_ENABLE_ENABLE_Enabled;

  nrfUart->EVENTS_RXDRDY = 0x0UL;
  nrfUart->EVENTS_TXDRDY = 0x0UL;

  nrfUart->TASKS_STARTRX = 0x1UL;
  nrfUart->TASKS_STARTTX = 0x1UL;

  nrfUart->INTENSET = UART_INTENSET_RXDRDY_Msk;

  NVIC_ClearPendingIRQ(IRQn);
  NVIC_SetPriority(IRQn, 3);
  NVIC_EnableIRQ(IRQn);
}

void Uart::end()
{
  NVIC_DisableIRQ(IRQn);

  nrfUart->INTENCLR = UART_INTENCLR_RXDRDY_Msk;

  nrfUart->TASKS_STOPRX = 0x1UL;
  nrfUart->TASKS_STOPTX = 0x1UL;

  nrfUart->ENABLE = UART_ENABLE_ENABLE_Disabled;

  nrfUart->PSELTXD = 0xFFFFFFFF;
  nrfUart->PSELRXD = 0xFFFFFFFF;

  nrfUart->PSELRTS = 0xFFFFFFFF;
  nrfUart->PSELCTS = 0xFFFFFFFF;

  rxBuffer.clear();
}

void Uart::flush()
{
}

void Uart::IrqHandler()
{
  if (nrfUart->EVENTS_RXDRDY)
  {
    rxBuffer.store_char(nrfUart->RXD);

    nrfUart->EVENTS_RXDRDY = 0x0UL;
  }
}

int Uart::available()
{
  return rxBuffer.available();
}

int Uart::peek()
{
  return rxBuffer.peek();
}

int Uart::read()
{
  return rxBuffer.read_char();
}

size_t Uart::write(const uint8_t data)
{
  nrfUart->TXD = data;

  while(!nrfUart->EVENTS_TXDRDY);

  nrfUart->EVENTS_TXDRDY = 0x0UL;

  return 1;
}

#if defined(NRF52)
  #define NRF_UART0_IRQn UARTE0_UART0_IRQn
#elif defined(NRF51)
  #define NRF_UART0_IRQn UART0_IRQn
#endif

#if defined(PIN_SERIAL_CTS) && defined(PIN_SERIAL_RTS)
  Uart Serial( NRF_UART0, NRF_UART0_IRQn, PIN_SERIAL_RX, PIN_SERIAL_TX, PIN_SERIAL_CTS, PIN_SERIAL_RTS );
#else
  Uart Serial( NRF_UART0, NRF_UART0_IRQn, PIN_SERIAL_RX, PIN_SERIAL_TX );
#endif

#if defined(NRF52)
extern "C"
{
  void UARTE0_UART0_IRQHandler()
  {
    Serial.IrqHandler();
  }
}
#elif defined(NRF51)
extern "C"
{
  void UART0_IRQHandler()
  {
    Serial.IrqHandler();
  }
}
#endif
