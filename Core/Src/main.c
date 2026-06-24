/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * 教学说明：
 * 下方这些 hh 开头的头文件是用户自己写的控制层，和 CubeMX 生成的
 * adc/tim/usart/gpio 等外设初始化层分开。
 *
 * 本文件作为入口，把两层连接起来：
 * - CubeMX 层负责初始化 STM32 外设；
 * - hh 控制层负责读取传感器、运行 FOC、输出 PWM。
 */
#include "../hhSerial/hhSerial.h"
#include "../hhPwm/hhPwm.h"
#include "../hhFoc/hhFoc.h"
#include "../hhAS5600/hhAS5600.h"
#include "../hhGetIq/hhGetIq.h"
#include"math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
 * 1 ms 控制节拍标志：
 * TIM4 的中断真正发生在中断上下文中；回调里只把标志置 1，
 * 主循环看到标志后再执行传感器刷新和闭环控制。
 * 这种写法能避免在中断里运行较重的 FOC 计算。
 */
uint8_t  Flag_1ms=0;

/**
 * @brief HAL 定时器周期溢出回调。
 *
 * 所在流程：
 * 1. `hhFocInit()` 启动 TIM4 基本定时器中断；
 * 2. `TIM4_IRQHandler()` 调用 HAL 定时器中断处理；
 * 3. HAL 在更新事件发生后调用本函数；
 * 4. 本函数把 `Flag_1ms` 置 1；
 * 5. `main()` 主循环在非中断上下文执行真正控制计算。
 *
 * @param htim 触发回调的定时器句柄。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim4){
		Flag_1ms=1;
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 串口打印分频计数器：主循环很快，不能每次循环都打印，否则串口会占满时间。 */
int CountOutPut = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /*
   * 用户控制层初始化：
   * - Power_supply 是母线电压，影响电压限幅和 PWM 换算；
   * - 极对数 7 用于机械角度 -> 电角度；
   * - 方向 -1 用于把 AS5600 读数方向修正成控制方向；
   * - 初始选择 SVPWM 调制。
   */
  float Power_supply =12.6;
  hhFocInit(Power_supply,7, -1,PWM_MODE_SVPWM);
  /*
   * 三级串级闭环参数：
   * 位置环输出目标速度，速度环输出目标 Iq，Iq 环输出 Uq。
   * 这里的 limit 注释可按控制量理解：
   * - 位置环 limit 限定目标速度；
   * - 速度环 limit 限定目标 Iq；
   * - Iq 环 limit 限定输出电压。
   */
  hhSetCloseLoopPos_PIDM0(1.0f, 0.0f, 0.0f,100000.0f,30.0f);//限定最大速度是30
  hhSetCloseLoopVel_PIDM0(0.02f, 1.0f, 0.0f,30.0f,0.5f);//限定最大力矩是0.5
  hhSetCloseLoopIq_PIDM0(3.0f, 20.0f, 0.0f,10000.0f,Power_supply/2);
  /* RetFloat1 保存串口收到的第一个浮点数，当前作为 M0 的目标位置。 */
  float RetFloat1;
//  float RetFloat2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	   * 串口目标值读取：
	   * `hhSerial.c` 的 USART1 空闲中断回调会更新 ReceivedFloat1，
	   * 这里每次主循环读取最新值，不需要阻塞等待串口。
	   */
	  RetFloat1= GetSerialRetFloat1();
	  if(Flag_1ms==1){
		  /*
		   * 1 ms 控制主流程：
		   * 1. 更新 AS5600 角度/速度和 ADC/Iq；
		   * 2. 运行 M0 的位置-速度-Iq 三级串级闭环；
		   * 3. 清除节拍标志，等待下一次 TIM4 中断。
		   */
		  UpdateAllSensor();
		  hhCloseLoopPos_WithVelIqM0(RetFloat1);
		  Flag_1ms=0;
	  }
	  CountOutPut++;
	  if (CountOutPut >= 10000) {
		/* 慢速打印调试量；当前打印 M0 滤波 Iq，也可以改成速度等变量观察闭环。 */
		Serial_Printf("%f\r\n", GetFilterIqM0());//GetFilterV_M0()
	    CountOutPut = 0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief 系统时钟配置。
  *
  * 本工程使用外部高速晶振 HSE，经 PLL 倍频到 72 MHz。
  * 这个 72 MHz 是后面所有时间尺度的基础：
  * - TIM4 用它分频得到 1 ms 控制节拍；
  * - TIM2/TIM3 用它产生 PWM 计数；
  * - ADC 时钟也从 APB2 分频得到。
  *
  * 如果修改这里的 PLL 或分频参数，要同步重新检查 TIM4 的 1 ms 周期、
  * PWM 频率和 ADC 采样速度。
  *
  * System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	//	野生绿波电龙�???518379509
	//	qq�???799382581
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
