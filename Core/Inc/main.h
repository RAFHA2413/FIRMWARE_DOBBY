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
#include "stm32h5xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_IN_Pin GPIO_PIN_13
#define SW_IN_GPIO_Port GPIOC
#define SIR1_Pin GPIO_PIN_0
#define SIR1_GPIO_Port GPIOC
#define SIR2_Pin GPIO_PIN_1
#define SIR2_GPIO_Port GPIOC
#define SIR3_Pin GPIO_PIN_2
#define SIR3_GPIO_Port GPIOC
#define SIR4_Pin GPIO_PIN_3
#define SIR4_GPIO_Port GPIOC
#define SIR5_Pin GPIO_PIN_0
#define SIR5_GPIO_Port GPIOA
#define SIR6_Pin GPIO_PIN_1
#define SIR6_GPIO_Port GPIOA
#define SIR7_Pin GPIO_PIN_2
#define SIR7_GPIO_Port GPIOA
#define SRI8_Pin GPIO_PIN_3
#define SRI8_GPIO_Port GPIOA
#define SIR9_Pin GPIO_PIN_4
#define SIR9_GPIO_Port GPIOA
#define SIR10_Pin GPIO_PIN_5
#define SIR10_GPIO_Port GPIOA
#define SIR11_Pin GPIO_PIN_6
#define SIR11_GPIO_Port GPIOA
#define SIR12_Pin GPIO_PIN_7
#define SIR12_GPIO_Port GPIOA
#define SIR13_Pin GPIO_PIN_4
#define SIR13_GPIO_Port GPIOC
#define SIR14_Pin GPIO_PIN_5
#define SIR14_GPIO_Port GPIOC
#define SIR15_Pin GPIO_PIN_0
#define SIR15_GPIO_Port GPIOB
#define SIR16_Pin GPIO_PIN_1
#define SIR16_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define ENC1A_Pin GPIO_PIN_9
#define ENC1A_GPIO_Port GPIOE
#define ENC1B_Pin GPIO_PIN_11
#define ENC1B_GPIO_Port GPIOE
#define LEDS_IR_Pin GPIO_PIN_15
#define LEDS_IR_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOD
#define LEDR1_Pin GPIO_PIN_10
#define LEDR1_GPIO_Port GPIOD
#define LEDR2_Pin GPIO_PIN_11
#define LEDR2_GPIO_Port GPIOD
#define PWM11_Pin GPIO_PIN_12
#define PWM11_GPIO_Port GPIOD
#define PWM12_Pin GPIO_PIN_13
#define PWM12_GPIO_Port GPIOD
#define PWM21_Pin GPIO_PIN_14
#define PWM21_GPIO_Port GPIOD
#define PWM22_Pin GPIO_PIN_15
#define PWM22_GPIO_Port GPIOD
#define NEOPIXEL_Pin GPIO_PIN_6
#define NEOPIXEL_GPIO_Port GPIOC
#define SD_DET_Pin GPIO_PIN_8
#define SD_DET_GPIO_Port GPIOA
#define ENC2B_Pin GPIO_PIN_15
#define ENC2B_GPIO_Port GPIOA
#define ENC2A_Pin GPIO_PIN_3
#define ENC2A_GPIO_Port GPIOB
#define STOP_Pin GPIO_PIN_8
#define STOP_GPIO_Port GPIOB
#define IR_in_Pin GPIO_PIN_9
#define IR_in_GPIO_Port GPIOB
#define IR_in_EXTI_IRQn EXTI9_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
