/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

//Test Git

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
#define WF_EN_Pin GPIO_PIN_3
#define WF_EN_GPIO_Port GPIOE
#define USART2_TX_WF_Pin GPIO_PIN_2
#define USART2_TX_WF_GPIO_Port GPIOA
#define USART2_RX_WF_Pin GPIO_PIN_3
#define USART2_RX_WF_GPIO_Port GPIOA
#define NSS_SPI_Pin GPIO_PIN_4
#define NSS_SPI_GPIO_Port GPIOC
#define RST_SPI_Pin GPIO_PIN_5
#define RST_SPI_GPIO_Port GPIOC
#define LORA_EN_Pin GPIO_PIN_11
#define LORA_EN_GPIO_Port GPIOE
#define BT_EN_Pin GPIO_PIN_12
#define BT_EN_GPIO_Port GPIOE
#define USART3_TX_LORA_Pin GPIO_PIN_8
#define USART3_TX_LORA_GPIO_Port GPIOD
#define USART3_RX_LORA_Pin GPIO_PIN_9
#define USART3_RX_LORA_GPIO_Port GPIOD
#define BTN_BOARD_Pin GPIO_PIN_15
#define BTN_BOARD_GPIO_Port GPIOD
#define USART6_TX_BT_Pin GPIO_PIN_6
#define USART6_TX_BT_GPIO_Port GPIOC
#define USART6_RX_BT_Pin GPIO_PIN_7
#define USART6_RX_BT_GPIO_Port GPIOC
#define USART1_TX_FTDI_Pin GPIO_PIN_9
#define USART1_TX_FTDI_GPIO_Port GPIOA
#define USART1_RX_FTDI_Pin GPIO_PIN_10
#define USART1_RX_FTDI_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
