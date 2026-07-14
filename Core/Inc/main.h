/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOUCH_CS_Pin GPIO_PIN_13
#define TOUCH_CS_GPIO_Port GPIOC
#define OUT1_Pin GPIO_PIN_0
#define OUT1_GPIO_Port GPIOF
#define OUT2_Pin GPIO_PIN_1
#define OUT2_GPIO_Port GPIOF
#define OUT3_Pin GPIO_PIN_2
#define OUT3_GPIO_Port GPIOF
#define OUT4_Pin GPIO_PIN_3
#define OUT4_GPIO_Port GPIOF
#define OUT5_Pin GPIO_PIN_4
#define OUT5_GPIO_Port GPIOF
#define OUT6_Pin GPIO_PIN_5
#define OUT6_GPIO_Port GPIOF
#define IN1_Pin GPIO_PIN_6
#define IN1_GPIO_Port GPIOF
#define IN2_Pin GPIO_PIN_7
#define IN2_GPIO_Port GPIOF
#define IN3_Pin GPIO_PIN_8
#define IN3_GPIO_Port GPIOF
#define ADC1_1_Pin GPIO_PIN_2
#define ADC1_1_GPIO_Port GPIOA
#define ADC2_2_Pin GPIO_PIN_3
#define ADC2_2_GPIO_Port GPIOA
#define TOUCH_CLK_Pin GPIO_PIN_0
#define TOUCH_CLK_GPIO_Port GPIOB
#define TOUCH_PEN_Pin GPIO_PIN_1
#define TOUCH_PEN_GPIO_Port GPIOB
#define TOUCH_MISO_Pin GPIO_PIN_2
#define TOUCH_MISO_GPIO_Port GPIOB
#define TOUCH_MOSI_Pin GPIO_PIN_11
#define TOUCH_MOSI_GPIO_Port GPIOF
#define SPI_FLASH_CS_Pin GPIO_PIN_14
#define SPI_FLASH_CS_GPIO_Port GPIOB
#define OUT7_Pin GPIO_PIN_11
#define OUT7_GPIO_Port GPIOD
#define OUT8_Pin GPIO_PIN_12
#define OUT8_GPIO_Port GPIOD
#define IN_GND_Pin GPIO_PIN_13
#define IN_GND_GPIO_Port GPIOD
#define IN4_Pin GPIO_PIN_6
#define IN4_GPIO_Port GPIOC
#define IN5_Pin GPIO_PIN_7
#define IN5_GPIO_Port GPIOC
#define IN6_Pin GPIO_PIN_8
#define IN6_GPIO_Port GPIOC
#define IN7_Pin GPIO_PIN_9
#define IN7_GPIO_Port GPIOC
#define IN8_Pin GPIO_PIN_10
#define IN8_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
