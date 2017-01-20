/**
  ******************************************************************************
  * @file    uart.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    22-July-2016
  * @brief   provide the UART interface
  *
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
/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f1xx_system
  * @{
  */

/** @addtogroup STM32F1xx_System_Private_Includes
  * @{
  */
#include "stm32f1xx.h"
#include "hw_config.h"
#include "uart.h"
#include "timer.h"
#include "digital_io.h"
#include "interrupt.h"
#include "variant_hal_config.h"

#include "string.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Defines
  * @{
  */

/// @brief number of received characters
#define EMUL_TIMER_PERIOD 100

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Macros
  * @{
  */
static void USART1_AF_Remap(void)       {__HAL_RCC_AFIO_CLK_ENABLE(); __HAL_AFIO_REMAP_USART1_DISABLE();}
static void USART2_AF_Remap(void)       {__HAL_RCC_AFIO_CLK_ENABLE(); __HAL_AFIO_REMAP_USART2_DISABLE();}

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Variables
  * @{
  */
/// @brief uart caracteristics
UART_HandleTypeDef g_UartHandle[NB_UART_MANAGED];
static const uart_conf_t g_uart_config[NB_UART_MANAGED] = UART_PARAM;
static uart_param_t g_uart_param[NB_UART_MANAGED];

static UART_Emul_HandleTypeDef g_UartEmulHandle[NB_UART_EMUL_MANAGED];
static const uart_emul_conf_t g_uartEmul_config[NB_UART_EMUL_MANAGED] = UART_EMUL_PARAM;
static uart_emul_param_t g_uart_emul_param[NB_UART_EMUL_MANAGED];

//@brief just a simple buffer for the uart reception
uint8_t g_rx_data[1];

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_FunctionPrototypes
  * @{
  */
uart_id_e get_uart_id_from_handle(UART_HandleTypeDef *huart);

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Functions
  * @{
  */

/**
  * @brief  This function returns the corresponding uart id function of the
  *         handle
  * @param  huart : one of the defined serial interface
  * @retval the UART id
  */
uart_id_e get_uart_id_from_handle(UART_HandleTypeDef *huart)
{
  uart_id_e uart_id = NB_UART_MANAGED;
  int i;
  for(i = 0; i<NB_UART_MANAGED; i++) {
    if(&g_UartHandle[i] == huart) {
      uart_id = i;
      break;
    }
  }
  return uart_id;
}



/**
  * @brief  UART MSP Initialization - perform IOs and clock init
  * @param  huart : one of the defined serial interface
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  uart_id_e uart_id;

  uart_id = get_uart_id_from_handle(huart);
  if(NB_UART_MANAGED == uart_id) {
    return;
  }

  // Enable GPIO TX/RX clock
  SET_GPIO_CLK(g_uart_config[uart_id].tx_port);
  SET_GPIO_CLK(g_uart_config[uart_id].rx_port);

  // Enable USART clock
  ENABLE_UART_CLK(huart->Instance);

  g_uart_config[uart_id].uart_af_remap();

  //##-2- Configure peripheral GPIO ##########################################

  // UART TX GPIO pin configuration
  HAL_GPIO_Init(g_uart_config[uart_id].tx_port,
                (GPIO_InitTypeDef *) &g_uart_config[uart_id].tx_pin);

  // UART RX GPIO pin configuration
  HAL_GPIO_Init(g_uart_config[uart_id].rx_port,
                (GPIO_InitTypeDef *) &g_uart_config[uart_id].rx_pin);

  HAL_NVIC_SetPriority(g_uart_config[uart_id].irqtype, 0, 1);
  HAL_NVIC_EnableIRQ(g_uart_config[uart_id].irqtype);
}

/**
  * @brief  UART MSP DeInitialization - perform IOs and clock deinit
  * @param  huart : one of the defined serial interface
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  uart_id_e uart_id;

  uart_id = get_uart_id_from_handle(huart);
  if(NB_UART_MANAGED == uart_id) {
    return;
  }

  DISABLE_UART_CLK(huart->Instance);

  HAL_GPIO_DeInit(g_uart_config[uart_id].tx_port, g_uart_config[uart_id].tx_pin.Pin);
  HAL_GPIO_DeInit(g_uart_config[uart_id].rx_port, g_uart_config[uart_id].rx_pin.Pin);
}

/**
  * @brief  Function called to initialize the uart interface
  * @param  serial_id : one of the defined serial interface
  * @param  baudRate : baudrate to apply to the uart
  * @retval None
  */
void uart_init(uart_id_e uart_id, uint32_t baudRate)
{
  if(uart_id>=NB_UART_MANAGED) {
    return;
  }

  g_UartHandle[uart_id].Instance          = g_uart_config[uart_id].usart_typedef;
  g_UartHandle[uart_id].Init.BaudRate     = baudRate;
  g_UartHandle[uart_id].Init.WordLength   = UART_WORDLENGTH_8B;
  g_UartHandle[uart_id].Init.StopBits     = UART_STOPBITS_1;
  g_UartHandle[uart_id].Init.Parity       = UART_PARITY_NONE;
  g_UartHandle[uart_id].Init.Mode         = UART_MODE_TX_RX;
  g_UartHandle[uart_id].Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  g_UartHandle[uart_id].Init.OverSampling = UART_OVERSAMPLING_16;

  memset(g_uart_param[uart_id].rxpData, 0, UART_RCV_SIZE);
  g_uart_param[uart_id].data_available = 0;
  g_uart_param[uart_id].begin = 0;
  g_uart_param[uart_id].end = 0;

  // g_UartHandle[uart_id].Init.OverSampling = UART_OVERSAMPLING_16;
  // g_UartHandle[uart_id].Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  if(HAL_UART_DeInit(&g_UartHandle[uart_id]) != HAL_OK)
  {
    return;
  }  
  if(HAL_UART_Init(&g_UartHandle[uart_id])!= HAL_OK) {
    return;
  }

  if(HAL_UART_Receive_IT(&g_UartHandle[uart_id], g_rx_data, 1)!= HAL_OK) {
    return;
  }
}

/**
  * @brief  Function called to deinitialize the uart interface
  * @param  serial_id : one of the defined serial interface
  * @retval None
  */
void uart_deinit(uart_id_e uart_id)
{
  HAL_UART_DeInit(&g_UartHandle[uart_id]);
}

/**
  * @brief  Function returns the amount of data available
  * @param  serial_id : one of the defined serial interface
  * @retval The number of serial data available - int
  */
int uart_available(uart_id_e uart_id)
{
  return g_uart_param[uart_id].data_available;
}

/**
  * @brief  Return the first element of the rx buffer
  * @param  serial_id : one of the defined serial interface
  * @retval The first byte of incoming serial data available (or -1 if no data is available) - int
  */
int8_t uart_read(uart_id_e uart_id)
{
  int8_t data = -1;

  if(g_uart_param[uart_id].data_available > 0) {

    data = g_uart_param[uart_id].rxpData[g_uart_param[uart_id].begin++];

    if(g_uart_param[uart_id].begin >= UART_RCV_SIZE) {
      g_uart_param[uart_id].begin = 0;
    }

    g_uart_param[uart_id].data_available--;
  }

  return data;
}

/**
  * @brief  Return the first element of the rx buffer without removing it from
  *         the buffer
  * @param  serial_id : one of the defined serial interface
  * @retval The first byte of incoming serial data available (or -1 if no data is available) - int
  */
int8_t uart_peek(uart_id_e uart_id)
{
  int8_t data = -1;

  if(g_uart_param[uart_id].data_available > 0) {
    data = g_uart_param[uart_id].rxpData[g_uart_param[uart_id].begin];
  }

  return data;
}

/**
  * @brief  Flush the content of the RX buffer
  * @param  serial_id : one of the defined serial interface
  * @retval None
  */
void uart_flush(uart_id_e uart_id)
{
  g_uart_param[uart_id].data_available = 0;
  g_uart_param[uart_id].end = 0;
  g_uart_param[uart_id].begin = 0;
}

/**
  * @brief  write the data on the uart
  * @param  serial_id : one of the defined serial interface
  * @param  data : byte to write
  * @retval The number of bytes written
  */
size_t uart_write(uart_id_e uart_id, uint8_t data)
{
  HAL_UART_Transmit(&g_UartHandle[uart_id], &data, 1, 5000);
  return 1;
}

/**
  * @brief  error callback from UART
  * @param  UartHandle pointer on the uart reference
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_id_e uart_id = get_uart_id_from_handle(huart);
  if(NB_UART_MANAGED == uart_id) {
    return;
  }

  uart_deinit(uart_id);
  uart_init(uart_id, g_UartHandle[uart_id].Init.BaudRate);
}

/**
  * @brief  Read receive byte from uart
  * @param  UartHandle pointer on the uart reference
  * @param  byte byte to read
  * @retval None
  */
static void uart_getc(uart_id_e uart_id, uint8_t byte)
{
  if((NB_UART_MANAGED == uart_id) ||
      (g_uart_param[uart_id].data_available >= UART_RCV_SIZE)) {
    return;
  }
  g_uart_param[uart_id].rxpData[g_uart_param[uart_id].end++] = byte;
  if(g_uart_param[uart_id].end >= UART_RCV_SIZE) {
    g_uart_param[uart_id].end = 0;
  }
  g_uart_param[uart_id].data_available++;
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle pointer on the uart reference
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_id_e uart_id = get_uart_id_from_handle(huart);

  HAL_UART_Receive_IT(&g_UartHandle[uart_id], g_rx_data, 1);

  uart_getc(uart_id, g_rx_data[0]);
}

/**
  * @brief UART waking up
  * @param  UartHandle pointer on the uart reference
  * @retval None
  */
void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);
}

/******************************* EMULATED UART ********************************/
/* @brief Callback for uart emulated for compatibility with timer callback */
static void uart_emul_timer_irq(timer_id_e timer_id) {
  UNUSED(timer_id);
  g_uart_emul_param[UART1_EMUL_E].uart_rx_irqHandle();
}


/**
  * @brief  Initializes the UART Emulation MSP.
  * @param  huart: UART Emulation Handle
  * @retval None
  */
void HAL_UART_Emul_MspInit(UART_Emul_HandleTypeDef *huart)
{
  UNUSED(huart);

  // Enable GPIO TX/RX clock
  SET_GPIO_CLK(g_uartEmul_config[UART1_EMUL_E].tx_port);
  SET_GPIO_CLK(g_uartEmul_config[UART1_EMUL_E].rx_port);

  // UART TX GPIO pin configuration
  HAL_GPIO_Init(g_uartEmul_config[UART1_EMUL_E].tx_port,
                (GPIO_InitTypeDef *) &g_uartEmul_config[UART1_EMUL_E].tx_pin);

  // UART RX GPIO pin configuration
  HAL_GPIO_Init(g_uartEmul_config[UART1_EMUL_E].rx_port,
                (GPIO_InitTypeDef *) &g_uartEmul_config[UART1_EMUL_E].rx_pin);
}

/**
 * @brief  UART Emulation MSP DeInit.
 * @param  huart: UART Emulation handle
 * @retval None
 */
void HAL_UART_Emul_MspDeInit(UART_Emul_HandleTypeDef *huart)
{
  UNUSED(huart);

  HAL_GPIO_DeInit(g_uartEmul_config[UART1_EMUL_E].tx_port, g_uartEmul_config[UART1_EMUL_E].tx_pin.Pin);
  HAL_GPIO_DeInit(g_uartEmul_config[UART1_EMUL_E].rx_port, g_uartEmul_config[UART1_EMUL_E].rx_pin.Pin);
}

/**
  * @brief  Function called to initialize the emulated uart interface
  * @param  serial_id : one of the defined serial interface
  * @param  baudRate : baudrate to apply to the uart : 4800 or 9600 //TODO bug if baud rate > 9600
  * @retval None
  */
void uart_emul_init(uart_emul_id_e uart_id, uint32_t baudRate)
{
  if(uart_id>=NB_UART_EMUL_MANAGED) {
    return;
  }

  g_UartEmulHandle[uart_id].Init.Mode         = UART_EMUL_MODE_TX_RX;
  g_UartEmulHandle[uart_id].Init.BaudRate     = baudRate;
  g_UartEmulHandle[uart_id].Init.WordLength   = UART_EMUL_WORDLENGTH_8B;
  g_UartEmulHandle[uart_id].Init.StopBits     = UART_EMUL_STOPBITS_1;
  g_UartEmulHandle[uart_id].Init.Parity       = UART_EMUL_PARITY_NONE;
  g_UartEmulHandle[uart_id].Init.RxPinNumber  = g_uartEmul_config[uart_id].rx_pin.Pin;
  g_UartEmulHandle[uart_id].Init.TxPinNumber  = g_uartEmul_config[uart_id].tx_pin.Pin;
  g_UartEmulHandle[uart_id].RxPortName        = g_uartEmul_config[uart_id].rx_port;
  g_UartEmulHandle[uart_id].TxPortName        = g_uartEmul_config[uart_id].tx_port;

  memset(g_uart_emul_param[uart_id].rxpData, 0, UART_RCV_SIZE);
  g_uart_emul_param[uart_id].data_available = 0;
  g_uart_emul_param[uart_id].begin = 0;
  g_uart_emul_param[uart_id].end = 0;

  if(HAL_UART_Emul_Init(&g_UartEmulHandle[uart_id])!= HAL_OK) {
    return;
  }

  if (HAL_UART_Emul_Receive(&g_UartEmulHandle[uart_id], g_rx_data, 1) != HAL_OK) {
    return;
  }

  uart_emul_flush(uart_id);
}

/**
  * @brief  Function called to deinitialize the emulated uart interface
  * @param  serial_id : one of the defined serial interface
  * @retval None
  */
void uart_emul_deinit(uart_emul_id_e uart_id)
{
  if(uart_id>=NB_UART_EMUL_MANAGED) {
    return;
  }

  HAL_UART_Emul_DeInit(&g_UartEmulHandle[uart_id]);
}

/**
  * @brief  Function returns the amount of data available
  * @param  serial_id : one of the defined serial interface
  * @retval The number of serial data available - int
  */
int uart_emul_available(uart_emul_id_e uart_id)
{
  if(uart_id>=NB_UART_EMUL_MANAGED) {
    return 0;
  }

  return g_uart_emul_param[uart_id].data_available;
}

/**
  * @brief  Return the first element of the rx buffer
  * @param  serial_id : one of the defined serial interface
  * @retval The first byte of incoming serial data available (or -1 if no data is available) - int
  */
int8_t uart_emul_read(uart_emul_id_e uart_id)
{
  int8_t data = -1;

  if(uart_id>=NB_UART_EMUL_MANAGED) {
    return data;
  }

  if(g_uart_emul_param[uart_id].data_available > 0) {

    data = g_uart_emul_param[uart_id].rxpData[g_uart_emul_param[uart_id].begin++];

    if(g_uart_emul_param[uart_id].begin >= UART_RCV_SIZE) {
      g_uart_emul_param[uart_id].begin = 0;
    }

    g_uart_emul_param[uart_id].data_available--;
  }

  return data;
}

/**
  * @brief  write the data on the uart
  * @param  serial_id : one of the defined serial interface
  * @param  data : byte to write
  * @retval The number of bytes written
  */
size_t uart_emul_write(uart_emul_id_e uart_id, uint8_t data)
{
  if(uart_id>=NB_UART_EMUL_MANAGED) {
    return 0;
  }

  while(HAL_UART_Emul_Transmit(&g_UartEmulHandle[uart_id], &data, 1) != HAL_OK);
  return 1;
}

/**
  * @brief  Return the first element of the rx buffer without removing it from
  *         the buffer
  * @param  serial_id : one of the defined serial interface
  * @retval The first byte of incoming serial data available (or -1 if no data is available) - int
  */
int8_t uart_emul_peek(uart_emul_id_e uart_id)
{
  int8_t data = -1;

  if(uart_id>=NB_UART_EMUL_MANAGED) {
    return data;
  }

  if(g_uart_emul_param[uart_id].data_available > 0) {
    data = g_uart_emul_param[uart_id].rxpData[g_uart_emul_param[uart_id].begin];
  }

  return data;
}

/**
  * @brief  Flush the content of the RX buffer
  * @param  serial_id : one of the defined serial interface
  * @retval None
  */
void uart_emul_flush(uart_emul_id_e uart_id)
{
  if(uart_id>=NB_UART_EMUL_MANAGED) {
    return;
  }

  g_uart_emul_param[uart_id].data_available = 0;
  g_uart_emul_param[uart_id].end = 0;
  g_uart_emul_param[uart_id].begin = 0;
}

/**
  * @brief  Read receive byte from uart
  * @param  UartHandle : pointer on the uart reference
  * @param  byte : byte to read
  * @retval None
  */
static void uart_emul_getc(uart_emul_id_e uart_id, uint8_t byte)
{
  if((uart_id >= NB_UART_EMUL_MANAGED) ||
      (g_uart_emul_param[uart_id].data_available >= UART_RCV_SIZE)) {
    return;
  }

  g_uart_emul_param[uart_id].rxpData[g_uart_emul_param[uart_id].end++] = byte;
  if(g_uart_emul_param[uart_id].end >= UART_RCV_SIZE) {
    g_uart_emul_param[uart_id].end = 0;
  }
  g_uart_emul_param[uart_id].data_available++;
}

/**
  * @brief
  * @param  irq : pointer to function to call
  * @retval None
  */
void uart_emul_attached_handler(void (*irqHandle)(void))
{
  TimerHandleInit(TIM4_E, EMUL_TIMER_PERIOD - 1, (uint16_t)(HAL_RCC_GetHCLKFreq() / 1000) - 1); //50ms
  g_uart_emul_param[UART1_EMUL_E].uart_rx_irqHandle = irqHandle;
  attachIntHandle(TIM4_E, uart_emul_timer_irq);
}

/**
  * @brief  Initializes the UART Emulation Transfer Complete.
  * @param  huart: UART Emulation Handle
  * @retval None
  */
void HAL_UART_Emul_RxCpltCallback(UART_Emul_HandleTypeDef *huart)
{
  uart_emul_getc(UART1_EMUL_E, *huart->pRxBuffPtr);
  HAL_UART_Emul_Receive(&g_UartEmulHandle[UART1_EMUL_E], g_rx_data, 1);

  if(g_uart_emul_param[UART1_EMUL_E].uart_rx_irqHandle != NULL) {
    if(uart_emul_available(UART1_EMUL_E) < (UART_RCV_SIZE / 2))
      setTimerCounter(TIM4_E, 0);
    else if(uart_emul_available(UART1_EMUL_E) < (UART_RCV_SIZE/4*3))
      setTimerCounter(TIM4_E, EMUL_TIMER_PERIOD - 1);
    else
      g_uart_emul_param[UART1_EMUL_E].uart_rx_irqHandle();
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
