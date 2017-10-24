/**
  ******************************************************************************
  * @file    interrupt.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    22-July-2016
  * @brief   provide an interface to enable/disable interruptions
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
#include "interrupt.h"

#ifdef __cplusplus
 extern "C" {
#endif
/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_TypesDefinitions
  * @{
  */

/*As we can have only one interrupt/pin id, don't need to get the port info*/
typedef struct {
  uint32_t pin;
  uint32_t irqnb;
  void (*callback)(void);
  uint32_t mode;
  uint32_t configured;
}gpio_irq_conf_str;

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_Defines
  * @{
  */
#define NB_EXTI   (16)

#define GPIO_NUMBER ((uint32_t)16)
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
static gpio_irq_conf_str gpio_irq_conf[NB_EXTI] = {
  {.pin = GPIO_PIN_0,   .irqnb = EXTI0_IRQn,    .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_1,   .irqnb = EXTI1_IRQn,    .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_2,   .irqnb = EXTI2_IRQn,    .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_3,   .irqnb = EXTI3_IRQn,    .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_4,   .irqnb = EXTI4_IRQn,    .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_5,   .irqnb = EXTI9_5_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_6,   .irqnb = EXTI9_5_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_7,   .irqnb = EXTI9_5_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_8,   .irqnb = EXTI9_5_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_9,   .irqnb = EXTI9_5_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_10,  .irqnb = EXTI15_10_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_11,  .irqnb = EXTI15_10_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_12,  .irqnb = EXTI15_10_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_13,  .irqnb = EXTI15_10_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_14,  .irqnb = EXTI15_10_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 },
  {.pin = GPIO_PIN_15,  .irqnb = EXTI15_10_IRQn,  .callback = NULL, .mode = GPIO_MODE_IT_RISING, .configured = 0 }
};

/**
  * @}
  */

/** @addtogroup STM32F1xx_System_Private_FunctionPrototypes
  * @{
  */

static uint8_t get_pin_id(uint16_t pin);

/**
  * @}
  */
/**
  * @brief  This function returns the pin ID function of the HAL PIN definition
  * @param  pin : one of the gpio pin
  * @retval None
  */
uint8_t get_pin_id(uint16_t pin)
{
  uint8_t id = 0;

  while(pin != 0x0001) {
    pin=pin>>1;
    id++;
  }

  return id;
}
/**
  * @brief  This function enable the interruption on the selected port/pin
  * @param  port : one of the gpio port
  * @param  pin : one of the gpio pin
  **@param  callback : callback to call when the interrupt falls
  * @param  mode : one of the supported interrupt mode defined in stm32_hal_gpio
  * @retval None
  */
void stm32_interrupt_enable(GPIO_TypeDef *port, uint16_t pin,
                                  void (*callback)(void), uint32_t mode)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  volatile uint32_t *CRxRegister;
  uint32_t CRxRegOffset = 0;
  uint32_t ODRRegOffset = 0;
  uint32_t ConfigMask = 0x00000008; //MODE0 == 0x0 && CNF0 == 0x2
  uint8_t position;
  uint8_t id = get_pin_id(pin);

  // GPIO pin configuration
  GPIO_InitStruct.Pin       = pin;
  GPIO_InitStruct.Mode      = mode;

  //read the pull mode directly in the register as no function exists to get it.
  //Do it in case the user already defines the IO through the digital io
  //interface
  CRxRegister = (pin < GPIO_PIN_8) ? &port->CRL : &port->CRH;

  for (position = 0; position < GPIO_NUMBER; position++) {
    if(pin == (0x0001 << position)) {
      CRxRegOffset = (pin < GPIO_PIN_8) ? (position << 2) : ((position - 8) << 2);
      ODRRegOffset = position;
    }
  }

  if((*CRxRegister & ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << CRxRegOffset)) == (ConfigMask << CRxRegOffset)) {
    if((port->ODR & (GPIO_ODR_ODR0 << ODRRegOffset)) == (GPIO_ODR_ODR0 << ODRRegOffset)) {
      GPIO_InitStruct.Pull = GPIO_PULLUP;
    } else {
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
  } else {
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  }

  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(port, &GPIO_InitStruct);

  gpio_irq_conf[id].callback = callback;

  // Enable and set Button EXTI Interrupt to the lowest priority
  HAL_NVIC_SetPriority(gpio_irq_conf[id].irqnb, 0x06, 0);
  HAL_NVIC_EnableIRQ(gpio_irq_conf[id].irqnb);
}

/**
  * @brief  This function disable the interruption on the selected port/pin
  * @param  port : one of the gpio port
  * @param  pin : one of the gpio pin
  * @retval None
  */
void stm32_interrupt_disable(GPIO_TypeDef *port, uint16_t pin)
{
  uint8_t id = get_pin_id(pin);
  
  gpio_irq_conf[id].callback = NULL;
  
  for(int i = 0; i < NB_EXTI; i++) {
    if (gpio_irq_conf[id].irqnb == gpio_irq_conf[i].irqnb 
        && gpio_irq_conf[i].callback != NULL) {
      return;
    }
  }
  HAL_NVIC_DisableIRQ(gpio_irq_conf[id].irqnb);
}

/**
  * @brief This function his called by the HAL if the IRQ is valid
  * @param  GPIO_Pin : one of the gpio pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t irq_id = get_pin_id(GPIO_Pin);

  if(gpio_irq_conf[irq_id].callback != NULL) {
    gpio_irq_conf[irq_id].callback();
  }
}

/**
  * @brief This function handles external line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
  * @brief This function handles external line 1 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
  * @brief This function handles external line 2 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
  * @brief This function handles external line 3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

/**
  * @brief This function handles external line 4 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/**
  * @brief This function handles external line 5 to 9 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  uint32_t pin;
  for(pin = GPIO_PIN_5; pin <= GPIO_PIN_9; pin=pin<<1) {
    HAL_GPIO_EXTI_IRQHandler(pin);
  }
}

/**
  * @brief This function handles external line 10 to 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  uint32_t pin;
  for(pin = GPIO_PIN_10; pin <= GPIO_PIN_15; pin=pin<<1) {
    HAL_GPIO_EXTI_IRQHandler(pin);
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
