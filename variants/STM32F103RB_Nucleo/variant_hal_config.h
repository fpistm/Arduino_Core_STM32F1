/**
  ******************************************************************************
  * @file    variant_hal_config.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    16-Juanuary-2017
  * @brief   Customize the hardware configuration in this file.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

#ifndef __VARIANT_HAL_CONFIG_H
#define __VARIANT_HAL_CONFIG_H

#include "stm32f1xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

/******************************************************************************/
/* Serial wire JTAG */
// STM32F103 serial wire JTAG configuration must be disabled to use PA13, PA14,
// PA15, PB3 and PB4 pins. Comment the following line to enable the JTAG.
#define DISABLE_JTAG  1

/******************************************************************************/
/* ANALOG CHANNEL CONFIGURATION */

/*
WARNING: D5 and D9 can't be use in PWM mode in same time due to the internal
mapping of the STM32F103.
*/

/* Number of Analog pins: ADC, DAC, PWM */
#define NB_ANALOG_CHANNELS   12

/* Analog pin configuration:
Global: .port=GPIOx
        .pin=GPIO_PIN_x
ADC option: .adcInstance=ADCx
            .Channel=ADC_CHANNEL_x
DAC option: .dacInstance=DACx
            .dacChannelConf={ .DAC_Trigger=DAC_TRIGGER_x,
                              .DAC_OutputBuffer=DAC_OUTPUTBUFFER_x}
PWM option: .timInstance=TIMx
            .timChannel=TIM_CHANNEL_x
            .useNchannel=0 or 1 (if TIMx_CHxN)
*/
#define ANALOG_PARAM {                \
  {                                   \
    .port = GPIOA,                    \
    .pin = GPIO_PIN_0,                \
    .adcInstance = ADC1,              \
    .Channel = ADC_CHANNEL_0,         \
    .timInstance = NULL               \
  },                                  \
  {                                   \
    .port = GPIOA,                    \
    .pin = GPIO_PIN_1,                \
    .adcInstance = ADC1,              \
    .Channel = ADC_CHANNEL_1,         \
    .timInstance = NULL               \
  },                                  \
  {                                   \
    .port = GPIOA,                    \
    .pin = GPIO_PIN_4,                \
    .adcInstance = ADC1,              \
    .Channel = ADC_CHANNEL_4,         \
    .timInstance = NULL               \
  },                                  \
  {                                   \
    .port = GPIOA,                    \
    .pin = GPIO_PIN_7,                \
    .alFunction = TIM1_AF_PartialRemap, \
    .adcInstance = NULL,              \
    .timInstance = TIM1,              \
    .timChannel = TIM_CHANNEL_1,      \
    .useNchannel = 1                  \
  },                                  \
  {                                   \
    .port = GPIOB,                    \
    .pin = GPIO_PIN_0,                \
    .adcInstance = ADC1,              \
    .Channel = ADC_CHANNEL_8,         \
    .timInstance = NULL               \
  },                                  \
  {                                   \
    .port = GPIOB,                    \
    .pin = GPIO_PIN_3,                \
    .alFunction = TIM2_AF_FullRemap,  \
    .adcInstance = NULL,              \
    .timInstance = TIM2,              \
    .timChannel = TIM_CHANNEL_2,      \
    .useNchannel = 0                  \
  },                                  \
  {                                   \
    .port = GPIOB,                    \
    .pin = GPIO_PIN_4,                \
    .alFunction = TIM3_AF_PartialRemap, \
    .adcInstance = NULL,              \
    .timInstance = TIM3,              \
    .timChannel = TIM_CHANNEL_1,      \
    .useNchannel = 0                  \
  },                                  \
  {                                   \
    .port = GPIOB,                    \
    .pin = GPIO_PIN_6,                \
    .alFunction = TIM4_AF_NoRemap,    \
    .adcInstance = NULL,              \
    .timInstance = TIM4,              \
    .timChannel = TIM_CHANNEL_1,      \
    .useNchannel = 0                  \
  },                                  \
  {                                   \
    .port = GPIOB,                    \
    .pin = GPIO_PIN_10,               \
    .alFunction = TIM2_AF_FullRemap,  \
    .adcInstance = NULL,              \
    .timInstance = TIM2,              \
    .timChannel = TIM_CHANNEL_3,      \
    .useNchannel = 0                  \
  },                                  \
  {                                   \
    .port = GPIOC,                    \
    .pin = GPIO_PIN_0,                \
    .adcInstance = ADC1,              \
    .Channel = ADC_CHANNEL_10,        \
    .timInstance = NULL               \
  },                                  \
  {                                   \
    .port = GPIOC,                    \
    .pin = GPIO_PIN_1,                \
    .adcInstance = ADC1,              \
    .Channel = ADC_CHANNEL_11,        \
    .timInstance = NULL               \
  },                                  \
  {                                   \
    .port = GPIOC,                    \
    .pin = GPIO_PIN_7,                \
    .alFunction = TIM3_AF_FullRemap,  \
    .adcInstance = NULL,              \
    .timInstance = TIM3,              \
    .timChannel = TIM_CHANNEL_2,      \
    .useNchannel = 0                  \
  }                                   \
}

//Enable GPIO port clock
#define SET_GPIO_CLK(gpiox) do {  \
  if((gpiox) == GPIOA) {          \
    __GPIOA_CLK_ENABLE();         \
  } else if((gpiox) == GPIOB){    \
    __GPIOB_CLK_ENABLE();         \
  } else if((gpiox) == GPIOC){    \
    __GPIOC_CLK_ENABLE();         \
  }                               \
} while(0)

//Enable ADC clock
#define SET_ADC_CLK(adc) do {       \
  if((adc) == ADC1) {               \
    __HAL_RCC_ADC1_CLK_ENABLE();    \
  }                                 \
  /*else if((adc) == ADC2) {        \
    __HAL_RCC_ADC2_CLK_ENABLE();    \
  } else if((adc) == ADC3) {        \
    __HAL_RCC_ADC3_CLK_ENABLE();    \
  }                                */ \
} while(0)

//Disable ADC clock
#define RESET_ADC_CLK(adc) do {     \
  if((adc) == ADC1) {               \
    __HAL_RCC_ADC1_CLK_DISABLE();   \
  }                                 \
  /*else if((adc) == ADC2) {        \
    __HAL_RCC_ADC2_CLK_DISABLE();   \
  } else if((adc) == ADC3) {        \
    __HAL_RCC_ADC3_CLK_DISABLE();   \
  }                                 */\
} while(0)

/******************************************************************************/
/* GPIO external interrupt configuration */
/* Number of pin connected to an external interrupt by port */
#define NB_GPIO_EXTI   16

/* List of EXTI connected to pin 0 to 15 */
#define GPIO_EXTI_PARAM {   \
  EXTI0_IRQn,               \
  EXTI1_IRQn,               \
  EXTI2_IRQn,               \
  EXTI3_IRQn,               \
  EXTI4_IRQn,               \
  EXTI9_5_IRQn,             \
  EXTI9_5_IRQn,             \
  EXTI9_5_IRQn,             \
  EXTI9_5_IRQn,             \
  EXTI9_5_IRQn,             \
  EXTI15_10_IRQn,           \
  EXTI15_10_IRQn,           \
  EXTI15_10_IRQn,           \
  EXTI15_10_IRQn,           \
  EXTI15_10_IRQn,           \
  EXTI15_10_IRQn            \
}

#ifdef __cplusplus
}
#endif

#endif /* __VARIANT_HAL_CONFIG_H */
