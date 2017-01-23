/**
  ******************************************************************************
  * @file    analog.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    22-July-2016
  * @brief   Header for analog module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ANALOG_H
#define __ANALOG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
typedef struct {
  GPIO_TypeDef  *port;
  uint32_t pin;
  void (*alFunction)(void);
  ADC_TypeDef *adcInstance;
  uint32_t Channel;

#if defined (STM32F100xB) || defined (STM32F100xE) || defined (STM32F101xE) || defined (STM32F101xG) || defined (STM32F103xE) || defined (STM32F103xG) || defined (STM32F105xC) || defined (STM32F107xC)
  DAC_TypeDef *dacInstance;
  uint32_t dacChannel;
  DAC_ChannelConfTypeDef dacChannelConf;
#endif

  TIM_TypeDef *timInstance;
  uint32_t timChannel;
  uint32_t useNchannel;
}analog_config_str;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void dac_write_value(GPIO_TypeDef  *port, uint32_t pin, uint32_t value, uint8_t do_init);
void dac_stop(GPIO_TypeDef  *port, uint32_t pin);
uint16_t adc_read_value(GPIO_TypeDef  *port, uint32_t pin, uint8_t do_init);
void pwm_start(GPIO_TypeDef  *port, uint32_t pin, uint32_t clock_freq, uint32_t period, uint32_t value, uint8_t do_init);
void pwm_stop(GPIO_TypeDef  *port, uint32_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __ANALOG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
