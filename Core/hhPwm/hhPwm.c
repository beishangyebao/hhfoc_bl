/*
 * hhPwm.c
 *
 *  Created on: Mar 11, 2025
 *      Author: KingPC
 */

#include "tim.h"

/**
 * @brief 启动全部 PWM 输出通道。
 *
 * 所在流程：
 * `hhFocInit()` 初始化 FOC 时调用本函数，随后 `hhFoc.c` 才能通过
 * `M0SetPwmX()` / `M1SetPwmX()` 或 `M0SVPWMSetPwmX()` /
 * `M1SVPWMSetPwmX()` 实时更新三相输出。
 *
 * 当前工程一共输出 6 路 PWM：
 * - M0 三相：TIM2_CH1、TIM2_CH2、TIM2_CH3；
 * - M1 三相：TIM2_CH4、TIM3_CH1、TIM3_CH2。
 *
 * @note 这里只启动 PWM，不计算占空比，也不设置电机方向。
 */
void InitPwm(){
	   /* M0 的 A/B/C 三相由 TIM2 的 1/2/3 通道输出。 */
	   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	   /* M1 的三相跨 TIM2 与 TIM3：A 相在 TIM2_CH4，B/C 相在 TIM3_CH1/CH2。 */
	   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

/**
 * @brief 按归一化占空比设置 M0 A 相 PWM。
 * @param Duty 占空比，0.0 表示 0%，1.0 表示接近 100%。
 * @note `hhFoc.c` 的 SPWM 分支会先把相电压换算成占空比，再调用本函数。
 */
void M0SetPwmA(float Duty){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Duty * htim2.Init.Period);
}

/**
 * @brief 按归一化占空比设置 M0 B 相 PWM。
 * @param Duty 占空比，0.0 表示 0%，1.0 表示接近 100%。
 */
void M0SetPwmB(float Duty){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Duty * htim2.Init.Period);
}

/**
 * @brief 按归一化占空比设置 M0 C 相 PWM。
 * @param Duty 占空比，0.0 表示 0%，1.0 表示接近 100%。
 */
void M0SetPwmC(float Duty){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,Duty * htim2.Init.Period);
}

/**
 * @brief 按归一化占空比设置 M1 A 相 PWM。
 * @param Duty 占空比，0.0 表示 0%，1.0 表示接近 100%。
 */
void M1SetPwmA(float Duty){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,Duty * htim2.Init.Period);
}

/**
 * @brief 按归一化占空比设置 M1 B 相 PWM。
 * @param Duty 占空比，0.0 表示 0%，1.0 表示接近 100%。
 * @note 当前代码使用 `htim2.Init.Period` 作为换算基准，即使输出通道在 TIM3。
 */
void M1SetPwmB(float Duty){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Duty * htim2.Init.Period);
}

/**
 * @brief 按归一化占空比设置 M1 C 相 PWM。
 * @param Duty 占空比，0.0 表示 0%，1.0 表示接近 100%。
 * @note 当前代码使用 `htim2.Init.Period` 作为换算基准，即使输出通道在 TIM3。
 */
void M1SetPwmC(float Duty){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Duty * htim2.Init.Period);
}

//SVPWM
/**
 * @brief 直接写 M0 A 相的 CCR 比较值。
 * @param CCR SVPWM 算法算出的比较值，单位是定时器计数。
 * @note SVPWM 分支已经算出定时器计数值，不再经过占空比乘周期。
 */
void M0SVPWMSetPwmA(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,CCR);
}

/**
 * @brief 直接写 M0 B 相的 CCR 比较值。
 * @param CCR SVPWM 算法算出的比较值，单位是定时器计数。
 */
void M0SVPWMSetPwmB(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,CCR);
}

/**
 * @brief 直接写 M0 C 相的 CCR 比较值。
 * @param CCR SVPWM 算法算出的比较值，单位是定时器计数。
 */
void M0SVPWMSetPwmC(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,CCR);
}

/**
 * @brief 直接写 M1 A 相的 CCR 比较值。
 * @param CCR SVPWM 算法算出的比较值，单位是定时器计数。
 */
void M1SVPWMSetPwmA(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,CCR);
}

/**
 * @brief 直接写 M1 B 相的 CCR 比较值。
 * @param CCR SVPWM 算法算出的比较值，单位是定时器计数。
 */
void M1SVPWMSetPwmB(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,CCR);
}

/**
 * @brief 直接写 M1 C 相的 CCR 比较值。
 * @param CCR SVPWM 算法算出的比较值，单位是定时器计数。
 */
void M1SVPWMSetPwmC(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,CCR);
}
