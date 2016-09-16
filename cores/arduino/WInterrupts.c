/*
  Copyright (c) 2011-2012 Arduino.  All right reserved.

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

#include "WInterrupts.h"

#ifdef __cplusplus
 extern "C" {
#endif

//This is the list of the digital IOs configured
PinDescription g_intPinConfigured[MAX_DIGITAL_IOS];


void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode)
{
  int i;
  uint32_t it_mode;

  //not a valid pin
  if(pin>MAX_DIGITAL_IOS) {
    return ;
  }

  //find the pin.
  for(i = 0; i < NB_PIN_DESCRIPTIONS; i++) {
    if(g_APinDescription[i].arduino_id == pin) {
      g_intPinConfigured[pin] = g_APinDescription[i];
      g_intPinConfigured[pin].configured = true;
      break;
    }
  }

  switch(mode) {

    case CHANGE :
      it_mode = GPIO_MODE_IT_RISING_FALLING;
    break;

    case FALLING :
    case LOW :
      it_mode = GPIO_MODE_IT_FALLING;
    break;
    case RISING :
    case HIGH :
      it_mode = GPIO_MODE_IT_RISING;
    break;
    default:
      it_mode = GPIO_MODE_IT_RISING;
    break;
  }

  stm32_interrupt_enable(g_intPinConfigured[pin].ulPort,
                          g_intPinConfigured[pin].ulPin, callback, it_mode);

}

void detachInterrupt(uint32_t pin)
{
  int i;

  //not a valid pin
  if(pin>MAX_DIGITAL_IOS) {
    return ;
  }

  //find the pin.
  for(i = 0; i < NB_PIN_DESCRIPTIONS; i++) {
    if(g_APinDescription[i].arduino_id == pin) {
      g_intPinConfigured[pin] = g_APinDescription[i];
      g_intPinConfigured[pin].configured = true;
      break;
    }
  }

  stm32_interrupt_disable(g_intPinConfigured[pin].ulPort,
                          g_intPinConfigured[pin].ulPin);
}
