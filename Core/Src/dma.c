/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
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
#include "dma.h"

/* USER CODE BEGIN 0 */
/*
 * DMA 学习重点：
 * 这里的 `MX_DMA_Init()` 只打开 DMA1 控制器时钟。
 * 具体 DMA 通道参数不在本函数里配置，而是在各外设 MSP 初始化中配置：
 * - `adc.c` 的 `HAL_ADC_MspInit()` 配置 DMA1_Channel1 给 ADC1；
 * - `usart.c` 的 `HAL_UART_MspInit()` 配置 DMA1_Channel4 给 USART1_TX。
 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @brief 打开 DMA1 控制器时钟。
  *
  * 如果不先打开 DMA 控制器时钟，后续 ADC/USART 的 DMA 通道初始化无法正常工作。
  *
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

