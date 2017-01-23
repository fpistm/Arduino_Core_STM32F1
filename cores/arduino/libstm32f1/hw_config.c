/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    22-July-2016
  * @brief   provide some hw interface for the Arduino interface
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
#include "variant_hal_config.h"

#ifdef SERIAL_USB
extern PCD_HandleTypeDef hpcd_USB_FS;
#endif

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Defines
  * @{
  */
/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_FunctionPrototypes
  * @{
  */

extern void SystemClock_Config(void);

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Functions
  * @{
  */
/**
  * @brief  System Clock Configuration : 64MHz
  * @param  None
  * @retval None
  */

/**
  * @brief  This function performs the global init of the system (HAL, IOs...)
  * @param  None
  * @retval None
  */
void hw_config_init(void)
{
  //Initialize the HAL
  HAL_Init();

#ifdef DISABLE_JTAG
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_DISABLE();
#endif /* DISABLE_JTAG */

  // Configure the system clock
  SystemClock_Config();
}

/**
  * @brief  This function enable the GPIO clock
  * @param  GPIO_TypeDef GPIOx
  * @retval None
  */

//Enable GPIO port clock
void set_gpio_clk(GPIO_TypeDef *GPIOx) {
  if(GPIOx == NULL)
    return;

  if(GPIOx == GPIOA) {
    __GPIOA_CLK_ENABLE();
  } else if(GPIOx == GPIOB){
    __GPIOB_CLK_ENABLE();
  } else if(GPIOx == GPIOC){
    __GPIOC_CLK_ENABLE();
  } else if(GPIOx == GPIOD){
    __GPIOD_CLK_ENABLE();
  }
#ifdef GPIOE
  else if(GPIOx == GPIOE){
    __GPIOE_CLK_ENABLE();
  }
#endif /* GPIOE */
#ifdef GPIOF
  else if(GPIOx == GPIOF){
    __GPIOF_CLK_ENABLE();
  }
#endif /* GPIOF */
#ifdef GPIOG
  else if(GPIOx == GPIOG){
    __GPIOG_CLK_ENABLE();
  }
#endif /* GPIOG */
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
