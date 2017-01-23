/**
  ******************************************************************************
  * @file    uart.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    22-July-2016
  * @brief   Header for uart module
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
#ifndef __UART_H
#define __UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "uart_emul.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
typedef enum {
  UART1_EMUL_E = 0,
  NB_UART_EMUL_MANAGED
} uart_emul_id_e;

typedef enum {
  NATIVE_UART_E   = 0,
  EMULATED_UART_E = 1,
  NB_UART_OPTION
} uart_option_e;

/* Exported constants --------------------------------------------------------*/
#define UART_RCV_SIZE 128

/// @brief defines the global attributes of the UART
typedef struct {
  USART_TypeDef * usart_typedef;
  IRQn_Type       irqtype;
  GPIO_TypeDef  *tx_port;
  GPIO_InitTypeDef tx_pin;
  GPIO_TypeDef  *rx_port;
  GPIO_InitTypeDef rx_pin;
  void (*uart_af_remap)(void);
}uart_conf_t;

typedef struct {
  uint8_t rxpData[UART_RCV_SIZE];
  volatile uint32_t data_available;
  volatile uint8_t begin;
  volatile uint8_t end;
}uart_param_t;

typedef struct {
  UART_Emul_TypeDef uartEmul_typedef;
  GPIO_TypeDef  *tx_port;
  GPIO_InitTypeDef tx_pin;
  GPIO_TypeDef  *rx_port;
  GPIO_InitTypeDef rx_pin;
}uart_emul_conf_t;

typedef struct {
  void (*uart_rx_irqHandle)(void);
  uint8_t rxpData[UART_RCV_SIZE];
  volatile uint32_t data_available;
  volatile uint8_t begin;
  volatile uint8_t end;
}uart_emul_param_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void uart_init(uart_id_e uart_id, uint32_t baudRate);
void uart_deinit(uart_id_e uart_id);
int uart_available(uart_id_e uart_id);
int8_t uart_peek(uart_id_e uart_id);
int8_t uart_read(uart_id_e uart_id);
void uart_flush(uart_id_e uart_id);
size_t uart_write(uart_id_e uart_id, uint8_t data);

void uart_emul_init(uart_emul_id_e uart_id, uint32_t baudRate);
void uart_emul_deinit(uart_emul_id_e uart_id);
int uart_emul_available(uart_emul_id_e uart_id);
int8_t uart_emul_read(uart_emul_id_e uart_id);
size_t uart_emul_write(uart_emul_id_e uart_id, uint8_t data);
int8_t uart_emul_peek(uart_emul_id_e uart_id);
void uart_emul_flush(uart_emul_id_e uart_id);
void uart_emul_attached_handler(void (*irqHandle)(void));

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
