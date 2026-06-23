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
char Serial_RxPacket[100];
void StartSerialITReceive() {
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*) Serial_RxPacket,
			sizeof(Serial_RxPacket)); //最后参数表示最大接收长度
}
float ReceivedFloat1 = 0;
float ReceivedFloat2 = 0;
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
float GetSerialRetFloat1() {
	return ReceivedFloat1;
}
float GetSerialRetFloat2() {
	return ReceivedFloat2;
}

