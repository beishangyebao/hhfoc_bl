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
#include "stm32f1xx_hal.h"

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
#define M0SCL_Pin GPIO_PIN_10
#define M0SCL_GPIO_Port GPIOB
#define M0SDA_Pin GPIO_PIN_11
#define M0SDA_GPIO_Port GPIOB
#define PB4_Pin GPIO_PIN_4
#define PB4_GPIO_Port GPIOB
#define M1SCL_Pin GPIO_PIN_6
#define M1SCL_GPIO_Port GPIOB
#define M1SDA_Pin GPIO_PIN_7
#define M1SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/**
 * @brief 本工程的用户引脚命名说明。
 *
 * CubeMX 在上方生成的 `M0SCL_Pin/M0SDA_Pin/M1SCL_Pin/M1SDA_Pin`
 * 是两颗 AS5600 软件 I2C 的 GPIO：
 * - M0 AS5600：PB10 = SCL，PB11 = SDA；
 * - M1 AS5600：PB6  = SCL，PB7  = SDA。
 *
 * `PB4_Pin` 当前配置为上拉输入，本次控制主线没有直接使用。
 *
 * 这些宏是所有硬件映射的源头之一，阅读 `hhAS5600.c` 和 `gpio.c`
 * 时要回到这里核对引脚含义。
 */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
