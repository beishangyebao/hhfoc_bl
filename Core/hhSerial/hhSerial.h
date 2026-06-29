
#ifndef HHSERIAL_HHSERIAL_H_
#define HHSERIAL_HHSERIAL_H_

/**
 * @file hhSerial.h
 * @brief USART1 调试和目标值输入接口。
 *
 * 本模块承担两个角色：
 * - 输出：`Serial_Printf()` 把传感器、电流或调试信息发给上位机；
 * - 输入：USART1 空闲中断接收字符串，将 `"目标1"` 或 `"目标1,目标2"`
 *   解析为浮点数，供 `main.c` 的控制循环读取。
 */

/**
 * @brief 启动 USART1 “接收到空闲帧即回调”的中断接收。
 * @note 当前由 `hhFocInit()` 调用一次，之后每次接收完成会在回调中重新启动。
 */
void StartSerialITReceive();

/**
 * @brief 通过 USART1 打印格式化字符串。
 *
 * 用法类似 `printf()`，常用于把电流、速度、角度等变量发回串口助手。
 * 头文件保留当前工程原有声明形式；实际定义在 `.c` 文件中带可变参数。
 */
void Serial_Printf();

/**
 * @brief 获取最近一次串口接收到的第一个浮点数。
 * @return 第一个目标值；在本工程主循环中作为 M0 位置目标使用。
 */
float GetSerialRetFloat1();

/**
 * @brief 获取最近一次串口接收到的第二个浮点数。
 * @return 第二个目标值；如果本次只接收一个数，则回调中会置为 0。
 */
float GetSerialRetFloat2();

#endif /* HHSERIAL_HHSERIAL_H_ */
