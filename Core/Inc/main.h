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
#define S2_INT_Pin GPIO_PIN_0
#define S2_INT_GPIO_Port GPIOC
#define S2_XS_Pin GPIO_PIN_1
#define S2_XS_GPIO_Port GPIOC
#define S1_INT_Pin GPIO_PIN_2
#define S1_INT_GPIO_Port GPIOC
#define S1_SX_Pin GPIO_PIN_3
#define S1_SX_GPIO_Port GPIOC
#define M0_PWM0_Pin GPIO_PIN_0
#define M0_PWM0_GPIO_Port GPIOA
#define M0_PWM1_Pin GPIO_PIN_1
#define M0_PWM1_GPIO_Port GPIOA
#define M1_PWM1_Pin GPIO_PIN_2
#define M1_PWM1_GPIO_Port GPIOA
#define S0_INT_Pin GPIO_PIN_4
#define S0_INT_GPIO_Port GPIOC
#define S0_SX_Pin GPIO_PIN_5
#define S0_SX_GPIO_Port GPIOC
#define OPT_A_Pin GPIO_PIN_0
#define OPT_A_GPIO_Port GPIOB
#define OPT_B_Pin GPIO_PIN_1
#define OPT_B_GPIO_Port GPIOB
#define OPT_C_Pin GPIO_PIN_2
#define OPT_C_GPIO_Port GPIOB
#define OPT_D_Pin GPIO_PIN_10
#define OPT_D_GPIO_Port GPIOB
#define I2C_RST_Pin GPIO_PIN_12
#define I2C_RST_GPIO_Port GPIOB
#define I2C_SEL0_Pin GPIO_PIN_13
#define I2C_SEL0_GPIO_Port GPIOB
#define I2C_SEL1_Pin GPIO_PIN_14
#define I2C_SEL1_GPIO_Port GPIOB
#define I2C_SEL2_Pin GPIO_PIN_15
#define I2C_SEL2_GPIO_Port GPIOB
#define S5_INT_Pin GPIO_PIN_8
#define S5_INT_GPIO_Port GPIOC
#define S5_XS_Pin GPIO_PIN_9
#define S5_XS_GPIO_Port GPIOC
#define MX_Fault_Pin GPIO_PIN_8
#define MX_Fault_GPIO_Port GPIOA
#define LED_7_Pin GPIO_PIN_15
#define LED_7_GPIO_Port GPIOA
#define LED_6_Pin GPIO_PIN_10
#define LED_6_GPIO_Port GPIOC
#define LED_5_Pin GPIO_PIN_11
#define LED_5_GPIO_Port GPIOC
#define LED_4_Pin GPIO_PIN_12
#define LED_4_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_2
#define LED_3_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_3
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_4
#define LED_1_GPIO_Port GPIOB
#define LED_0_Pin GPIO_PIN_5
#define LED_0_GPIO_Port GPIOB
#define S4_SX_Pin GPIO_PIN_6
#define S4_SX_GPIO_Port GPIOB
#define S4_INT_Pin GPIO_PIN_7
#define S4_INT_GPIO_Port GPIOB
#define S3_INT_Pin GPIO_PIN_8
#define S3_INT_GPIO_Port GPIOB
#define S3_XS_Pin GPIO_PIN_9
#define S3_XS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
