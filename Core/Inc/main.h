/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define JOYSTICK_SEL_Pin GPIO_PIN_13
#define JOYSTICK_SEL_GPIO_Port GPIOC
#define JOYSTICK_SEL_EXTI_IRQn EXTI15_10_IRQn
#define JOYSTICK_LEFT_Pin GPIO_PIN_4
#define JOYSTICK_LEFT_GPIO_Port GPIOC
#define JOYSTICK_LEFT_EXTI_IRQn EXTI4_IRQn
#define JOYSTICK_DOWN_Pin GPIO_PIN_5
#define JOYSTICK_DOWN_GPIO_Port GPIOC
#define JOYSTICK_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define LD3_Pin GPIO_PIN_1
#define LD3_GPIO_Port GPIOB
#define JOYSTICK_RIGHT_Pin GPIO_PIN_2
#define JOYSTICK_RIGHT_GPIO_Port GPIOB
#define JOYSTICK_RIGHT_EXTI_IRQn EXTI2_IRQn
#define JOYSTICK_UP_Pin GPIO_PIN_10
#define JOYSTICK_UP_GPIO_Port GPIOB
#define JOYSTICK_UP_EXTI_IRQn EXTI15_10_IRQn
#define BUCK_RED_DRIVE_Pin GPIO_PIN_6
#define BUCK_RED_DRIVE_GPIO_Port GPIOC
#define BUCK_GREEN_DRIVE_Pin GPIO_PIN_8
#define BUCK_GREEN_DRIVE_GPIO_Port GPIOC
#define BUCK_BLUE_DRIVE_Pin GPIO_PIN_8
#define BUCK_BLUE_DRIVE_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_15
#define LD2_GPIO_Port GPIOA
#define USART3_TX_Pin GPIO_PIN_10
#define USART3_TX_GPIO_Port GPIOC
#define USART3_RX_Pin GPIO_PIN_11
#define USART3_RX_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_5
#define LD5_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_7
#define LD4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
