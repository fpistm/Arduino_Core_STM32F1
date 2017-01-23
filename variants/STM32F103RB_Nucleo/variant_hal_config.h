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
Alternate function: .alFunction= TIMx_AF_NoRemap, TIMx_AF_FullRemap,
                                  TIMx_AF_PartialRemap,
                                  TIMx_AF_PartialRemap_1 or
                                  TIMx_AF_PartialRemap_2
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

/******************************************************************************/
/* I2C configuration */
/* I2C instance_e available. Number of I2C instance configured. */
typedef enum {
  I2C_1,
  //I2C_2,
  NB_I2C_INSTANCES
}i2c_instance_e;

/* I2C configuration:
  .i2c_instance=I2Cx
  .ev_irq : I2C Event interrupt handler
  .er_irq : I2C Error interrupt handler
  .i2c_alternate : enable I2C1 alternate function
                    Can be I2C1_AF_FullRemap or I2C1_AF_NoRemap
  .sda_port = GPIOx,
  .sda_pin = GPIO_PIN_x,
  .scl_port = GPIOx,
  .scl_pin = GPIO_PIN_x
*/
#define I2C_PARAM {                   \
  {                                   \
    .i2c_instance = I2C1,             \
    .ev_irq = I2C1_EV_IRQn,           \
    .er_irq = I2C1_ER_IRQn,           \
    .i2c_alternate = I2C1_AF_FullRemap,\
    .sda_port = GPIOB,                \
    .sda_pin = GPIO_PIN_9,            \
    .scl_port = GPIOB,                \
    .scl_pin = GPIO_PIN_8             \
  }                                   \
}

/*TBD: Wire library must be rework to accept more than one I2C without use this macro. */
#define LINK_I2C_INSTANCE_OBJ case I2C_1: ptr = &Wire; break; \
                              /*case I2C_2: ptr = &Wire1; break;*/

/******************************************************************************/
/* Timer configuration */
/* Timer instance_e available. Number of timer instance configured. */
typedef enum {
  TIM1_E,
  TIM2_E,
  TIM3_E,
  TIM4_E,
  NB_TIMER_MANAGED
} timer_id_e;

/* Timer configuration:
  .timInstance=TIMx
  .irqtype : Timer interrupt handler
  .timer_mode : Can be reserved for PWM or can be used for all
  .prescalerLimit : bits_8, bits_16 or bits_32
*/
#define TIMER_PARAM {           \
  {                             \
    .timInstance = TIM1,        \
    .irqtype = TIM1_UP_IRQn,    \
    .timer_mode = TIMER_OTHER,  \
    .prescalerLimit = bits_16   \
  },                            \
  {                             \
    .timInstance = TIM2,        \
    .irqtype = TIM2_IRQn,       \
    .timer_mode = TIMER_OTHER,  \
    .prescalerLimit = bits_16   \
  },                            \
  {                             \
    .timInstance = TIM3,        \
    .irqtype = TIM3_IRQn,       \
    .timer_mode = TIMER_OTHER,  \
    .prescalerLimit = bits_16   \
  },                            \
  {                             \
    .timInstance = TIM4,        \
    .irqtype = TIM4_IRQn,       \
    .timer_mode = TIMER_OTHER,  \
    .prescalerLimit = bits_16   \
  }                             \
}

/******************************************************************************/
/* Uart configuration */
/* UART instance_e available. Number of UART instance configured. */
typedef enum {
  USART1_E,
  USART2_E,
  NB_UART_MANAGED
} uart_id_e;

/* UART configuration:
.usart_typedef = USARTx
.irqtype : UART interrupt handler
.tx_port = GPIOx
.tx_pin : GPIO_InitTypeDef
.rx_port = GPIOx
.rx_pin : GPIO_InitTypeDef
.uart_af_remap : alternate function remapping
                Can be: USARTx_AF_NoRemap, USARTx_AF_FullRemap or USARTx_AF_PartialRemap
*/
#define UART_PARAM {                                                            \
  {                                                                             \
    .usart_typedef = USART1,                                                    \
    .irqtype = USART1_IRQn,                                                     \
    .tx_port = GPIOA,                                                           \
    .tx_pin = {GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH}, \
    .rx_port = GPIOA,                                                           \
    .rx_pin = {GPIO_PIN_10, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH},\
    .uart_af_remap = USART1_AF_NoRemap                                          \
  },                                                                            \
  {                                                                             \
    .usart_typedef = USART2,                                                    \
    .irqtype = USART2_IRQn,                                                     \
    .tx_port = GPIOA,                                                           \
    .tx_pin = {GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH}, \
    .rx_port = GPIOA,                                                           \
    .rx_pin = {GPIO_PIN_3, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH}, \
    .uart_af_remap = USART2_AF_NoRemap                                          \
  }                                                                             \
}

/* Emulated uart configuration */
#define UART_EMUL_PARAM {                                                           \
  {                                                                                 \
    .uartEmul_typedef = {UART1_EMUL_E},                                             \
    .tx_port = GPIOB,                                                               \
    .tx_pin = {GPIO_PIN_3, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH}, \
    .rx_port = GPIOA,                                                               \
    .rx_pin = {GPIO_PIN_10, GPIO_MODE_IT_FALLING, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH}\
  }                                                                                 \
}


/******************************************************************************/
/* SPI configuration */

/* SPI instance_e available. Number of SPI instance configured. */
typedef enum {
  SPI_1,
  NB_SPI_INSTANCES
}spi_instance_e;

/* SPI configuration:
.spi_instance = SPIx
.spi_alternate : SPI alternate function
                Can be: SPIx_AF_NoRemap or SPIx_AF_FullRemap
.mosi_port = GPIOx
.mosi_pin =  GPIO_PIN_x
.miso_port = GPIOx
.miso_pin = GPIO_PIN_x
.sck_port = GPIOx
.sck_pin = GPIO_PIN_x
*/
#define SPI_PARAM {                   \
  {                                   \
    .spi_instance = SPI1,             \
    .spi_alternate = SPI1_AF_NoRemap, \
    .mosi_port = GPIOA,               \
    .mosi_pin =  GPIO_PIN_7,          \
    .miso_port = GPIOA,               \
    .miso_pin = GPIO_PIN_6,           \
    .sck_port = GPIOA,                \
    .sck_pin = GPIO_PIN_5,            \
  }                                   \
}

#ifdef __cplusplus
}
#endif

#endif /* __VARIANT_HAL_CONFIG_H */
