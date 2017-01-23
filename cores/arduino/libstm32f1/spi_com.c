/**
  ******************************************************************************
  * @file    spi_com.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    22-July-2016
  * @brief   provide the SPI interface
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
#include "spi_com.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Defines
  * @{
  */

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
static void SPI1_AF_FullRemap(void) {__HAL_AFIO_REMAP_SPI1_ENABLE();}
static void SPI1_AF_NoRemap(void)   {__HAL_AFIO_REMAP_SPI1_DISABLE();}

#ifdef SPI3
static void SPI3_AF_FullRemap(void) {__HAL_AFIO_REMAP_SPI3_ENABLE();}
static void SPI3_AF_NoRemap(void)   {__HAL_AFIO_REMAP_SPI3_DISABLE();}
#endif /* SPI3 */


/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Variables
  * @{
  */

static const spi_init_info_t g_spi_init_info[NB_SPI_INSTANCES] = SPI_PARAM;
static spi_param_t g_spi_param[NB_SPI_INSTANCES];

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Functions
  * @{
  */
/**
  * @brief  SPI initialization function
  * @param  spi_id : one of the SPI ids
  * @param  speed : spi output speed
  * @param  mode : one of the spi modes
  * @param  msb : set to 1 in msb first
  * @retval None
  */
void spi_init(spi_instance_e spi_id, uint32_t speed, spi_mode_e mode, uint8_t msb)
{
  ///perform SPI initialization Here
  SPI_HandleTypeDef *spiHandle;

  if(spi_id >= NB_SPI_INSTANCES) {
    return;
  }

  //###-- Configure the SPI peripheral #######################################
  // Set the SPI parameters
  spiHandle           = &g_spi_param[spi_id].spiHandle;
  spiHandle->Instance = g_spi_init_info[spi_id].spi_instance;

  spiHandle->Init.Mode              = SPI_MODE_MASTER;
  spiHandle->Init.Direction         = SPI_DIRECTION_2LINES;
  spiHandle->Init.DataSize          = SPI_DATASIZE_8BIT;

  if((mode == SPI_MODE_0)||(mode == SPI_MODE_1)) {
    spiHandle->Init.CLKPolarity     = SPI_POLARITY_LOW;
  } else {
    spiHandle->Init.CLKPolarity     = SPI_POLARITY_HIGH;
  }

  if((mode == SPI_MODE_0)||(mode == SPI_MODE_2)) {
    spiHandle->Init.CLKPhase        = SPI_PHASE_1EDGE;
  } else {
    spiHandle->Init.CLKPhase        = SPI_PHASE_2EDGE;
  }

  spiHandle->Init.NSS               = SPI_NSS_SOFT;

  //As system is running at 64MHz, compute the speeds function of this
  if(speed >= SPI_SPEED_CLOCK_DIV2_MHZ) {
      spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  } else if(speed >= SPI_SPEED_CLOCK_DIV4_MHZ) {
      spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  } else if (speed >= SPI_SPEED_CLOCK_DIV8_MHZ) {
      spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  } else if (speed >= SPI_SPEED_CLOCK_DIV16_MHZ) {
      spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  } else if (speed >= SPI_SPEED_CLOCK_DIV32_MHZ) {
      spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  } else if (speed >= SPI_SPEED_CLOCK_DIV64_MHZ) {
      spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  } else if (speed >= SPI_SPEED_CLOCK_DIV128_MHZ) {
      spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  } else if (speed >= SPI_SPEED_CLOCK_DIV256_MHZ) {
      spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  } else {
      spiHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  }

  if(msb == 0) {
    spiHandle->Init.FirstBit        = SPI_FIRSTBIT_LSB;
  } else {
    spiHandle->Init.FirstBit        = SPI_FIRSTBIT_MSB;
  }

  spiHandle->Init.TIMode            = SPI_TIMODE_DISABLE;
  spiHandle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  spiHandle->Init.CRCPolynomial     = 7;

  HAL_SPI_Init(spiHandle);
}

/**
  * @brief Initialize the SPI MSP.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  uint8_t spi_id = NB_SPI_INSTANCES;

  for(uint8_t i = 0; i < NB_SPI_INSTANCES; i++) {
    if(g_spi_init_info[i].spi_instance == hspi->Instance) {
      spi_id = i;
    }
  }

  if(spi_id == NB_SPI_INSTANCES)
    return;

  if(g_spi_param[spi_id].init_done == 0) {

    //##-1- Enable peripherals and GPIO Clocks #################################
    set_gpio_clk(g_spi_init_info[spi_id].mosi_port);
    set_gpio_clk(g_spi_init_info[spi_id].miso_port);
    set_gpio_clk(g_spi_init_info[spi_id].sck_port);

    // Enable SPI clock
    if(hspi->Instance == SPI1) {
      __HAL_RCC_SPI1_CLK_ENABLE();
    }
#ifdef SPI2
    else if(hspi->Instance == SPI2) {
      __HAL_RCC_SPI2_CLK_ENABLE();
    }
#endif /* SPI2 */
#ifdef SPI3
    else if(hspi->Instance == SPI3) {
      __HAL_RCC_SPI3_CLK_ENABLE();
    }
#endif /* SPI3 */

    //##-2- Configure peripheral GPIO ##########################################
    if(g_spi_init_info[spi_id].spi_alternate != NULL) {
      __HAL_RCC_AFIO_CLK_ENABLE();
      g_spi_init_info[spi_id].spi_alternate();
    }

    // SPI SCK GPIO pin configuration
    GPIO_InitStructure.Pin       = g_spi_init_info[spi_id].sck_pin;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull      = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(g_spi_init_info[spi_id].sck_port, &GPIO_InitStructure);

    // SPI MISO GPIO pin configuration
    GPIO_InitStructure.Pin       = g_spi_init_info[spi_id].miso_pin;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull      = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(g_spi_init_info[spi_id].miso_port, &GPIO_InitStructure);

    // SPI MOSI GPIO pin configuration
    GPIO_InitStructure.Pin       = g_spi_init_info[spi_id].mosi_pin;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull      = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(g_spi_init_info[spi_id].mosi_port, &GPIO_InitStructure);

    g_spi_param[spi_id].init_done = 1;
  }
}

/**
  * @brief This function is implemented to deinitialize the SPI interface
  *        (IOs + SPI block)
  * @param  spi_id : one of the SPI ids
  * @retval None
  */
void spi_deinit(spi_instance_e spi_id)
{
  if(spi_id >= NB_SPI_INSTANCES) {
    return;
  }

  HAL_SPI_DeInit(&g_spi_param[spi_id].spiHandle);

  if(g_spi_init_info[spi_id].spi_instance == SPI1) {
    __HAL_RCC_SPI1_CLK_DISABLE();
  }
#ifdef SPI2
  else if(g_spi_init_info[spi_id].spi_instance == SPI2) {
    __HAL_RCC_SPI2_CLK_DISABLE();
  }
#endif /* SPI2 */
#ifdef SPI3
  else if(g_spi_init_info[spi_id].spi_instance == SPI3) {
    __HAL_RCC_SPI3_CLK_DISABLE();
  }
#endif /* SPI3 */

  HAL_GPIO_DeInit(g_spi_init_info[spi_id].sck_port,g_spi_init_info[spi_id].sck_pin);
  HAL_GPIO_DeInit(g_spi_init_info[spi_id].miso_port,g_spi_init_info[spi_id].miso_pin);
  HAL_GPIO_DeInit(g_spi_init_info[spi_id].mosi_port,g_spi_init_info[spi_id].mosi_pin);
  g_spi_param[spi_id].init_done = 0;
}

/**
  * @brief This function is implemented by user to send data over SPI interface
  * @param  spi_id : one of the SPI ids
  * @param  Data : data to be sent
  * @param  len : length in bytes of the data to be sent
  * @param  Timeout: Timeout duration in tick
  * @retval status of the send operation (0) in case of error
  */
spi_status_e spi_send(spi_instance_e spi_id, uint8_t *Data,
                      uint16_t len, uint32_t Timeout)
{
  spi_status_e ret = SPI_OK;
  HAL_StatusTypeDef hal_status;

  if((spi_id >= NB_SPI_INSTANCES)||(len == 0)) {
    return SPI_ERROR;
  }

  hal_status = HAL_SPI_Transmit(&g_spi_param[spi_id].spiHandle,
                      Data, len, Timeout);

  if(hal_status == HAL_TIMEOUT) {
    ret = SPI_TIMEOUT;
  } else if(hal_status != HAL_OK) {
    ret = SPI_ERROR;
  }
  return ret;
}

/**
  * @brief This function is implemented by user to send/receive data over
  *         SPI interface
  * @param  spi_id : one of the SPI ids
  * @param  tx_buffer : tx data to send before reception
  * @param  rx_buffer : data to receive
  * @param  len : length in byte of the data to send and receive
  * @param  Timeout: Timeout duration in tick
  * @retval status of the send operation (0) in case of error
  */
spi_status_e spi_transfer(spi_instance_e spi_id, uint8_t * tx_buffer,
                      uint8_t * rx_buffer, uint16_t len, uint32_t Timeout)
{
  spi_status_e ret = SPI_OK;
  HAL_StatusTypeDef hal_status;

  if((spi_id >= NB_SPI_INSTANCES)||(len == 0)) {
    return SPI_ERROR;
  }

  hal_status = HAL_SPI_TransmitReceive(&g_spi_param[spi_id].spiHandle, tx_buffer,
                            rx_buffer, len, Timeout);

  if(hal_status == HAL_TIMEOUT) {
    ret = SPI_TIMEOUT;
  } else if(hal_status != HAL_OK) {
    ret = SPI_ERROR;
  }

  return ret;
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
