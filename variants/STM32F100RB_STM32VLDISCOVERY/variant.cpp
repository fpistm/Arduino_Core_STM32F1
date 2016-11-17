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

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=

//  arduino_id  |     ulPin    |   ulPort | mode               |          configured
{
  { PA0,   	GPIO_PIN_0,  GPIOA, GPIO_PIN_IO|GPIO_PIN_ADC ,                    false },
  { PA1,   	GPIO_PIN_1,  GPIOA, GPIO_PIN_IO|GPIO_PIN_ADC ,                    false },
  { PA2,   	GPIO_PIN_2,  GPIOA, GPIO_PIN_IO|GPIO_PIN_UART_TX ,                false },
  { PA3,   	GPIO_PIN_3,  GPIOA, GPIO_PIN_IO|GPIO_PIN_UART_RX ,                false },
  { PA4,   	GPIO_PIN_4,  GPIOA, GPIO_PIN_IO|GPIO_PIN_ADC ,                    false },
  { PA5,  	GPIO_PIN_5,  GPIOA, GPIO_PIN_IO|GPIO_PIN_SPI_SCK,                 false },
  { PA6,  	GPIO_PIN_6,  GPIOA, GPIO_PIN_IO|GPIO_PIN_SPI_MISO ,               false },
  { PA7,  	GPIO_PIN_7,  GPIOA, GPIO_PIN_IO|GPIO_PIN_SPI_MOSI|GPIO_PIN_PWM,   false },
  { PA8,   	GPIO_PIN_8,  GPIOA, GPIO_PIN_IO,                                  false },
  { PA9,   	GPIO_PIN_9,  GPIOA, GPIO_PIN_IO,                                  false },
  { PA10,   GPIO_PIN_10, GPIOA, GPIO_PIN_IO,                                  false },
  { PA11,  	GPIO_PIN_11, GPIOA, GPIO_PIN_MORPHO_ONLY,                         false },
  { PA12,  	GPIO_PIN_12, GPIOA, GPIO_PIN_MORPHO_ONLY,                         false },
  { PA13,  	GPIO_PIN_13, GPIOA, GPIO_PIN_MORPHO_ONLY,                         false },
  { PA14,  	GPIO_PIN_14, GPIOA, GPIO_PIN_MORPHO_ONLY,                         false },
  { PA15,  	GPIO_PIN_15, GPIOA, GPIO_PIN_MORPHO_ONLY ,                        false },
  { PB0,   	GPIO_PIN_0,  GPIOB, GPIO_PIN_ADC,                                 false },
  { PB1,  	GPIO_PIN_1,  GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB2,  	GPIO_PIN_2,  GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB3,   	GPIO_PIN_3,  GPIOB, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { PB4,   	GPIO_PIN_4,  GPIOB, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { PB5,   	GPIO_PIN_5,  GPIOB, GPIO_PIN_IO,                                  false },
  { PB6,  	GPIO_PIN_6,  GPIOB, GPIO_PIN_IO|GPIO_PIN_PWM|GPIO_PIN_SPI_CS,     false },
  { PB7,  	GPIO_PIN_7,  GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB8,  	GPIO_PIN_8,  GPIOB, GPIO_PIN_IO|GPIO_PIN_I2C_SCL,                 false },
  { PB9,  	GPIO_PIN_9,  GPIOB, GPIO_PIN_IO|GPIO_PIN_I2C_SDA,                 false },
  { PB10,   GPIO_PIN_10, GPIOB, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { PB11,  	GPIO_PIN_11, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB12,  	GPIO_PIN_12, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB13,  	GPIO_PIN_13, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB14,  	GPIO_PIN_14, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PB15,  	GPIO_PIN_15, GPIOB, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC0,   	GPIO_PIN_0,  GPIOC, GPIO_PIN_IO|GPIO_PIN_ADC,                     false },
  { PC1,   	GPIO_PIN_1,  GPIOC, GPIO_PIN_IO|GPIO_PIN_ADC,                     false },
  { PC2,  	GPIO_PIN_2,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC3,  	GPIO_PIN_3,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC4,  	GPIO_PIN_4,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC5,  	GPIO_PIN_5,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC6,  	GPIO_PIN_6,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC7,   	GPIO_PIN_7,  GPIOC, GPIO_PIN_IO|GPIO_PIN_PWM,                     false },
  { PC8,  	GPIO_PIN_8,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC9,  	GPIO_PIN_9,  GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC10,  	GPIO_PIN_10, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC11,  	GPIO_PIN_11, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC12,  	GPIO_PIN_12, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC13,  	GPIO_PIN_13, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC14,  	GPIO_PIN_14, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false },
  { PC15,  	GPIO_PIN_15, GPIOC, GPIO_PIN_MORPHO_ONLY,                         false }
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

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
  
void __libc_init_array(void);


void init( void )
{
  hw_config_init();
}

#ifdef __cplusplus
}
#endif
