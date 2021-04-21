/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
  /* yFLAG ==>stm32f4xx_nucleo.h header file for the BSP Common driver (led, bp, com ...)*/
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_nucleo.h"
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
#define yCubeMX 5.2
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define STTS751_INT_Pin GPIO_PIN_1
#define STTS751_INT_GPIO_Port GPIOC
#define STTS751_INT_EXTI_IRQn EXTI1_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LIS2DW12_INT1_Pin GPIO_PIN_0
#define LIS2DW12_INT1_GPIO_Port GPIOB
#define LIS2DW12_INT1_EXTI_IRQn EXTI0_IRQn
#define LPS22HH_INT_Pin GPIO_PIN_10
#define LPS22HH_INT_GPIO_Port GPIOB
#define LPS22HH_INT_EXTI_IRQn EXTI15_10_IRQn
#define LIS2DW12_INT2_Pin GPIO_PIN_7
#define LIS2DW12_INT2_GPIO_Port GPIOC
#define LIS2DW12_INT2_EXTI_IRQn EXTI9_5_IRQn
#define BP2_Pin GPIO_PIN_8
#define BP2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LSM6DSO_INT2_Pin GPIO_PIN_4
#define LSM6DSO_INT2_GPIO_Port GPIOB
#define LSM6DSO_INT2_EXTI_IRQn EXTI4_IRQn
#define LSM6DSO_INT1_Pin GPIO_PIN_5
#define LSM6DSO_INT1_GPIO_Port GPIOB
#define LSM6DSO_INT1_EXTI_IRQn EXTI9_5_IRQn
#define I2C1scl_Pin GPIO_PIN_8
#define I2C1scl_GPIO_Port GPIOB
#define I2C1sda_Pin GPIO_PIN_9
#define I2C1sda_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define yPROG "F401RE_IKS01_Unicleo"
#define yVER  "v5.2d"
#define yDATE "xx-avr-2021"

#define yDBG 1		//many printf
#define yTRC 1		//printf coup par coup

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
