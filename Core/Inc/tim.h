/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN Private defines */
/**
 * @brief 定时器在本工程中的分工。
 *
 * - TIM2：PWM 输出主定时器，CH1/2/3 给 M0 A/B/C，CH4 给 M1 A；
 * - TIM3：PWM 输出辅助定时器，CH1/2 给 M1 B/C；
 * - TIM4：1 ms 基准定时器，触发 `HAL_TIM_PeriodElapsedCallback()`，
 *   主循环据此运行传感器刷新和闭环控制。
 */

/* USER CODE END Private defines */

/**
 * @brief 初始化 TIM2 PWM 输出通道。
 */
void MX_TIM2_Init(void);

/**
 * @brief 初始化 TIM3 PWM 输出通道。
 */
void MX_TIM3_Init(void);

/**
 * @brief 初始化 TIM4 1 ms 基准定时器。
 */
void MX_TIM4_Init(void);

/**
 * @brief HAL 定时器 GPIO 复用初始化入口。
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

