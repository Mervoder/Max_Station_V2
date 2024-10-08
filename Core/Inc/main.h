/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define CS_Pin GPIO_PIN_1
#define CS_GPIO_Port GPIOC
#define ADC_VCC_Pin GPIO_PIN_2
#define ADC_VCC_GPIO_Port GPIOC
#define TX2_GPS_Pin GPIO_PIN_2
#define TX2_GPS_GPIO_Port GPIOA
#define RX2_GPS_Pin GPIO_PIN_3
#define RX2_GPS_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOC
#define RX3_LORA_Pin GPIO_PIN_5
#define RX3_LORA_GPIO_Port GPIOC
#define M0_Pin GPIO_PIN_0
#define M0_GPIO_Port GPIOB
#define M1_Pin GPIO_PIN_1
#define M1_GPIO_Port GPIOB
#define FN_Pin GPIO_PIN_2
#define FN_GPIO_Port GPIOB
#define TX3_LORA_Pin GPIO_PIN_10
#define TX3_LORA_GPIO_Port GPIOB
#define SECINP_Pin GPIO_PIN_12
#define SECINP_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_9
#define BUTTON_GPIO_Port GPIOC
#define GATE_D_Pin GPIO_PIN_10
#define GATE_D_GPIO_Port GPIOC
#define GATE_C_Pin GPIO_PIN_11
#define GATE_C_GPIO_Port GPIOC
#define GATE_B_Pin GPIO_PIN_3
#define GATE_B_GPIO_Port GPIOB
#define GATE_A_Pin GPIO_PIN_4
#define GATE_A_GPIO_Port GPIOB
#define INT2_Pin GPIO_PIN_9
#define INT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
