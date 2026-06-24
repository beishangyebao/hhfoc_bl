/*
 * hhPwm.h
 *
 *  Created on: Mar 11, 2025
 *      Author: KingPC
 */

#ifndef HHPWM_HHPWM_H_
#define HHPWM_HHPWM_H_

/**
 * @file hhPwm.h
 * @brief PWM 输出封装层，对外提供“按电机和相位写 PWM”的接口。
 *
 * 本文件不负责计算 FOC 电压矢量，只负责把上层给出的占空比或定时器
 * CCR 比较值写入 TIM2/TIM3 对应通道。学习时可以把它看成
 * `hhFoc.c` 与 STM32 定时器 PWM 硬件之间的适配层。
 *
 * 通道关系：
 * - M0：TIM2_CH1/CH2/CH3 对应 A/B/C 三相；
 * - M1：TIM2_CH4、TIM3_CH1、TIM3_CH2 对应 A/B/C 三相。
 */

/**
 * @brief 启动两台电机使用到的所有 PWM 输出通道。
 * @note 必须在写占空比或 CCR 之前调用，当前由 `hhFocInit()` 间接调用。
 */
void InitPwm();

/**
 * @brief 以占空比形式设置 M0 A 相 PWM。
 * @param Duty 归一化占空比，理论范围 0.0~1.0。
 */
void M0SetPwmA(float Duty);

/**
 * @brief 以占空比形式设置 M0 B 相 PWM。
 * @param Duty 归一化占空比，理论范围 0.0~1.0。
 */
void M0SetPwmB(float Duty);

/**
 * @brief 以占空比形式设置 M0 C 相 PWM。
 * @param Duty 归一化占空比，理论范围 0.0~1.0。
 */
void M0SetPwmC(float Duty);

/**
 * @brief 以占空比形式设置 M1 A 相 PWM。
 * @param Duty 归一化占空比，理论范围 0.0~1.0。
 */
void M1SetPwmA(float Duty);

/**
 * @brief 以占空比形式设置 M1 B 相 PWM。
 * @param Duty 归一化占空比，理论范围 0.0~1.0。
 */
void M1SetPwmB(float Duty);

/**
 * @brief 以占空比形式设置 M1 C 相 PWM。
 * @param Duty 归一化占空比，理论范围 0.0~1.0。
 */
void M1SetPwmC(float Duty);

/**
 * @brief 以定时器 CCR 计数值形式设置 M0 A 相 PWM，供 SVPWM 直接写比较值使用。
 * @param CCR TIMx->CCR 的比较值，单位是定时器计数。
 */
void M0SVPWMSetPwmA(uint32_t CCR);

/**
 * @brief 以定时器 CCR 计数值形式设置 M0 B 相 PWM，供 SVPWM 直接写比较值使用。
 * @param CCR TIMx->CCR 的比较值，单位是定时器计数。
 */
void M0SVPWMSetPwmB(uint32_t CCR);

/**
 * @brief 以定时器 CCR 计数值形式设置 M0 C 相 PWM，供 SVPWM 直接写比较值使用。
 * @param CCR TIMx->CCR 的比较值，单位是定时器计数。
 */
void M0SVPWMSetPwmC(uint32_t CCR);

/**
 * @brief 以定时器 CCR 计数值形式设置 M1 A 相 PWM，供 SVPWM 直接写比较值使用。
 * @param CCR TIMx->CCR 的比较值，单位是定时器计数。
 */
void M1SVPWMSetPwmA(uint32_t CCR);

/**
 * @brief 以定时器 CCR 计数值形式设置 M1 B 相 PWM，供 SVPWM 直接写比较值使用。
 * @param CCR TIMx->CCR 的比较值，单位是定时器计数。
 */
void M1SVPWMSetPwmB(uint32_t CCR);

/**
 * @brief 以定时器 CCR 计数值形式设置 M1 C 相 PWM，供 SVPWM 直接写比较值使用。
 * @param CCR TIMx->CCR 的比较值，单位是定时器计数。
 */
void M1SVPWMSetPwmC(uint32_t CCR);
#endif /* HHPWM_HHPWM_H_ */
