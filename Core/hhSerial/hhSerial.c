/*
 * hhSerial.c
 *
 *  Created on: Mar 11, 2025
 *      Author: KingPC
 */


#include <stdarg.h>
#include "stdio.h"
#include"gpio.h"
#include"usart.h"
#include <string.h>
#include <stdlib.h>

/**
 * @brief 通过 USART1 发送格式化文本，供调试观察变量。
 *
 * 所在流程：
 * - `main.c` 周期性调用它打印 `GetFilterIqM0()` 等调试量；
 * - `hhFocInit()` 也用它提示初始化完成；
 * - 底层最终通过 `HAL_UART_Transmit()` 阻塞发送到 USART1 TX。
 *
 * @param format printf 风格格式字符串。
 * @param ... 与格式字符串匹配的可变参数。
 * @note 当前实现先等待 USART1 不处于发送忙状态，再使用阻塞发送。
 *       这样写简单可靠，但在高频打印时会占用主循环时间。
 */
void Serial_Printf(const char *format, ...) {
	va_list args;			// 定义参数列表变量
	va_start(args, format); // 从format位置开始接收参数表，放在arg里面

	char strBuf[256];				// 定义输出的字符串
	vsprintf(strBuf, format, args); // 使用vsprintf将格式化的数据写入缓冲区
	va_end(args);					// 结束可变参数的使用

	// 等待上次的数据发送完成，避免新的数据覆盖正在传输的数据，导致混乱
	while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
		// Wait for DMA transfer to complete
	}

//	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) strBuf, strlen(strBuf));
	HAL_UART_Transmit(&huart1, (uint8_t*) strBuf, strlen(strBuf), HAL_MAX_DELAY);
}

/* 串口接收缓冲区：USART1 的空闲中断接收会把一帧字符串放到这里。 */
char Serial_RxPacket[100];

/**
 * @brief 启动 USART1 空闲中断接收。
 *
 * 所在流程：
 * `hhFocInit()` 初始化时调用本函数；收到一帧数据后，
 * HAL 会进入 `HAL_UARTEx_RxEventCallback()`，回调末尾再次调用
 * `HAL_UARTEx_ReceiveToIdle_IT()`，形成连续接收。
 *
 * @note “ReceiveToIdle” 适合串口助手发送不定长命令：当线路空闲时认为一帧结束。
 */
void StartSerialITReceive() {
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*) Serial_RxPacket,
			sizeof(Serial_RxPacket)); //最后参数表示最大接收长度
}

/* 最近一次解析出的两个浮点数。main.c 通过 Getter 读取，避免直接访问全局变量。 */
float ReceivedFloat1 = 0;
float ReceivedFloat2 = 0;

/**
 * @brief USART1 接收到一帧数据后的 HAL 回调，负责把文本命令解析成浮点目标值。
 *
 * 数据格式：
 * - `"12.3\n"`：解析为 `ReceivedFloat1 = 12.3`，`ReceivedFloat2 = 0`；
 * - `"12.3,4.5\n"`：解析为两个浮点数。
 *
 * 所在流程：
 * 1. `StartSerialITReceive()` 或上一次回调重新启动接收；
 * 2. 串口收到数据并检测到空闲；
 * 3. HAL 调用本回调；
 * 4. 本回调截断换行符、按逗号拆分、更新 `ReceivedFloat1/2`；
 * 5. 重新启动下一帧接收；
 * 6. `main.c` 通过 `GetSerialRetFloat1()` 读取目标位置。
 *
 * @param huart 触发回调的 UART 句柄。
 * @param Size 本次收到的字节数，不包含手动追加的字符串结束符。
 * @note 当前缓冲区长度为 100；代码会在 `Serial_RxPacket[Size]` 写入 `'\0'`，
 *       因此学习时要注意 Size 等于缓冲区长度时的边界风险。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart1) {
		Serial_RxPacket[Size] = '\0'; // 由于C语言中的字符串都必须以'\0'为结束标志的，所以接收完后需加上这行
		// 检查是否包含换行符 `\n`，如果有则截断字符串
		char *newlinePos = strchr(Serial_RxPacket, '\n');
		if (newlinePos != NULL) {
			*newlinePos = '\0'; // 将 `\n` 替换为字符串结束符
		}
		// 检查是否包含逗号 `,`
		char *commaPos = strchr(Serial_RxPacket, ',');
		if (commaPos != NULL) {
			// 如果有逗号，则将逗号替换为字符串结束符，并分别解析两个浮点数
			*commaPos = '\0';
			ReceivedFloat1 = atof(Serial_RxPacket);
			ReceivedFloat2 = atof(commaPos + 1);
		} else {
			// 如果没有逗号，则只解析一个浮点数
			ReceivedFloat1 = atof(Serial_RxPacket);
			ReceivedFloat2 = 0; // 如果没有第二个浮点数，可以将 ReceivedFloat2 设置为 0 或其他默认值
		}
		// 重新启动 UART 接收
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*) Serial_RxPacket,
				sizeof(Serial_RxPacket));
	}
}

/**
 * @brief 返回最近一次串口命令解析得到的第一个浮点数。
 * @return 第一个浮点数，当前主循环把它作为 M0 的位置目标。
 */
float GetSerialRetFloat1() {
	return ReceivedFloat1;
}

/**
 * @brief 返回最近一次串口命令解析得到的第二个浮点数。
 * @return 第二个浮点数；单参数命令时该值为 0。
 */
float GetSerialRetFloat2() {
	return ReceivedFloat2;
}

