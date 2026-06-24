/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
/*
 * GPIO 学习重点：
 * 本工程没有使用硬件 I2C 外设，而是把 AS5600 的 SCL/SDA 配成普通 GPIO
 * 开漏输出，再由 `hhAS5600.c` 手动产生 I2C 起始、停止、读写和 ACK。
 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @brief 初始化所有本工程用到的 GPIO。
 *
 * 对 FOC 主线最重要的是 AS5600 软件 I2C 引脚：
 * - PB10/PB11：M0SCL/M0SDA；
 * - PB6/PB7：M1SCL/M1SDA。
 *
 * 这些引脚被配置为开漏输出，并且初始写高，符合 I2C 总线空闲时
 * SCL/SDA 均为高电平的要求。
 */
/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level
   * 软件 I2C 空闲状态释放总线，因此先把四根 SCL/SDA 全部置高。
   */
  HAL_GPIO_WritePin(GPIOB, M0SCL_Pin|M0SDA_Pin|M1SCL_Pin|M1SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
   * 开漏输出用于模拟 I2C：输出低电平时主动拉低，总线高电平依赖释放/上拉。
   */
  GPIO_InitStruct.Pin = M0SCL_Pin|M0SDA_Pin|M1SCL_Pin|M1SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin
   * PB4 当前作为上拉输入保留，本工程 FOC 主控制路径没有直接读取它。
   */
  GPIO_InitStruct.Pin = PB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PB4_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
