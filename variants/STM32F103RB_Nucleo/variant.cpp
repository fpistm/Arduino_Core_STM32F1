/*
  Copyright (c) 2011 Arduino.  All right reserved.

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

#include "variant.h"
#include "variant_it.h"

/* GPIO ID    | label
  PA0         | A0
  PA1         | A1
  PA2         | D1/TX
  PA3         | D0/RX
  PA4         | A2
  PA5         | D13/SCK
  PA6         | D12/MISO
  PA7         | D11/PWM/MOSI
  PA8         | D7
  PA9         | D8
  PA10        | D2
  PA11        |
  PA12        |
  PA13        |
  PA14        |
  PA15        |
  PB0         | A3
  PB1         |
  PB2         |
  PB3         | D3/PWM
  PB4         | D5/PWM
  PB5         | D4
  PB6         | D10/PWM/CS
  PB7         |
  PB8         | D15/SCL
  PB9         | D14/SDA
  PB10        | D6/PWM
  PB11        |
  PB12        |
  PB13        |
  PB14        |
  PB15        |
  PC0         | A5
  PC1         | A4
  PC2         |
  PC3         |
  PC4         |
  PC5         |
  PC6         |
  PC7         | D9/PWM
  PC8         |
  PC9         |
  PC10        |
  PC11        |
  PC12        |
  PC13        |
  PC14        |
  PC15        |

*/

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=

//  arduino_id  |     ulPin    |   ulPort | mode               |          configured
{
  { PA3,   GPIO_PIN_3,  GPIOA, GPIO_PIN_IO|GPIO_PIN_UART_RX ,                false },
  { PA2,   GPIO_PIN_2,  GPIOA, GPIO_PIN_IO|GPIO_PIN_UART_TX ,                false },
  { PA10,  GPIO_PIN_10, GPIOA, GPIO_PIN_IO,                                  false },
  { PB3,   GPIO_PIN_3,  GPIOB, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { PB5,   GPIO_PIN_5,  GPIOB, GPIO_PIN_IO,                                  false },
  { PB4,   GPIO_PIN_4,  GPIOB, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { PB10,  GPIO_PIN_10, GPIOB, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { PA8,   GPIO_PIN_8,  GPIOA, GPIO_PIN_IO,                                  false },
  { PA9,   GPIO_PIN_9,  GPIOA, GPIO_PIN_IO,                                  false },
  { PC7,   GPIO_PIN_7,  GPIOC, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { PB6,   GPIO_PIN_6,  GPIOB, GPIO_PIN_IO|GPIO_PIN_PWM|GPIO_PIN_SPI_CS,     false },
  { PA7,   GPIO_PIN_7,  GPIOA, GPIO_PIN_IO|GPIO_PIN_SPI_MOSI|GPIO_PIN_PWM,   false },
  { PA6,   GPIO_PIN_6,  GPIOA, GPIO_PIN_IO|GPIO_PIN_SPI_MISO ,               false },
  { PA5,   GPIO_PIN_5,  GPIOA, GPIO_PIN_IO|GPIO_PIN_SPI_SCK,                 false },
  { PB9,   GPIO_PIN_9,  GPIOB, GPIO_PIN_IO|GPIO_PIN_I2C_SDA,                 false },
  { PB8,   GPIO_PIN_8,  GPIOB, GPIO_PIN_IO|GPIO_PIN_I2C_SCL,                 false },
  { PA0,   GPIO_PIN_0,  GPIOA, GPIO_PIN_IO|GPIO_PIN_ADC ,                    false },
  { PA1,   GPIO_PIN_1,  GPIOA, GPIO_PIN_IO|GPIO_PIN_ADC ,                    false },
  { PA4,   GPIO_PIN_4,  GPIOA, GPIO_PIN_IO|GPIO_PIN_ADC ,                    false },
  { PB0,   GPIO_PIN_0,  GPIOB, GPIO_PIN_ADC,                                 false },
  { PC1,   GPIO_PIN_1,  GPIOC, GPIO_PIN_IO|GPIO_PIN_ADC,                     false },
  { PC0,   GPIO_PIN_0,  GPIOC, GPIO_PIN_IO|GPIO_PIN_ADC,                     false },
  { PA11,  GPIO_PIN_11, GPIOA, GPIO_PIN_MORPHO_ONLY,                         false },
  { PA12,  GPIO_PIN_12, GPIOA, GPIO_PIN_MORPHO_ONLY,                         false },
  { PA13,  GPIO_PIN_13, GPIOA, GPIO_PIN_MORPHO_ONLY,                         false },
  { PA14,  GPIO_PIN_14, GPIOA, GPIO_PIN_MORPHO_ONLY,                         false },
  { PA15,  GPIO_PIN_15, GPIOA, GPIO_PIN_MORPHO_ONLY ,                        false },
  { PB1,   GPIO_PIN_1,  GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB2,   GPIO_PIN_2,  GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB7,   GPIO_PIN_7,  GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB11,  GPIO_PIN_11, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB12,  GPIO_PIN_12, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB13,  GPIO_PIN_13, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB14,  GPIO_PIN_14, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB15,  GPIO_PIN_15, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC2,   GPIO_PIN_2,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC3,   GPIO_PIN_3,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC4,   GPIO_PIN_4,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC5,   GPIO_PIN_5,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC6,   GPIO_PIN_6,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC8,   GPIO_PIN_8,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC9,   GPIO_PIN_9,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC10,  GPIO_PIN_10, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC11,  GPIO_PIN_11, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC12,  GPIO_PIN_12, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC13,  GPIO_PIN_13, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC14,  GPIO_PIN_14, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC15,  GPIO_PIN_15, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false }
} ;

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */

UARTClass Serial(USART2_E);    //available on PA2/PA3
UARTClass Serial1(USART1_E);   //available on PA9/PA10
USARTClass Serial2(USART2_E);  //available on PA2/PA3

void serialEvent() __attribute__((weak));
void serialEvent() { }

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

/* System clock configured at 72MHz using external high speed clock from
 MCO output of ST-LINK MCU (8MHz) */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
  
void __libc_init_array(void);

uint32_t analogPinConvert(uint32_t ulPin) {
  if(ulPin < ARDUINO_PIN_A0) 
    return ulPin | ARDUINO_PIN_A0; 
  else 
    return ulPin;
}

void init( void )
{
  hw_config_init();
}

#ifdef __cplusplus
}
#endif
