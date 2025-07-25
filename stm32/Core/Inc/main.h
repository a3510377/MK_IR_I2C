/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define I2C_S1_Pin GPIO_PIN_0
#define I2C_S1_GPIO_Port GPIOF
#define I2C_S2_Pin GPIO_PIN_1
#define I2C_S2_GPIO_Port GPIOF
#define MUX_0_Pin GPIO_PIN_1
#define MUX_0_GPIO_Port GPIOA
#define MUX_1_Pin GPIO_PIN_2
#define MUX_1_GPIO_Port GPIOA
#define MUX_2_Pin GPIO_PIN_3
#define MUX_2_GPIO_Port GPIOA
#define MUX_3_Pin GPIO_PIN_4
#define MUX_3_GPIO_Port GPIOA
#define STATUS_Pin GPIO_PIN_5
#define STATUS_GPIO_Port GPIOA
#define D_CK_Pin GPIO_PIN_6
#define D_CK_GPIO_Port GPIOA
#define L_CK_Pin GPIO_PIN_7
#define L_CK_GPIO_Port GPIOA
#define LEDS_Pin GPIO_PIN_1
#define LEDS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
