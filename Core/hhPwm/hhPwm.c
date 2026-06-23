/*
 * hhPwm.c
 *
 *  Created on: Mar 11, 2025
 *      Author: KingPC
 */

#include "tim.h"
void InitPwm(){
	   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	   HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}
void M0SetPwmA(float Duty){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Duty * htim2.Init.Period);
}
void M0SetPwmB(float Duty){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Duty * htim2.Init.Period);
}
void M0SetPwmC(float Duty){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,Duty * htim2.Init.Period);
}

void M1SetPwmA(float Duty){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,Duty * htim2.Init.Period);
}
void M1SetPwmB(float Duty){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Duty * htim2.Init.Period);
}
void M1SetPwmC(float Duty){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Duty * htim2.Init.Period);
}

//SVPWM
void M0SVPWMSetPwmA(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,CCR);
}
void M0SVPWMSetPwmB(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,CCR);
}
void M0SVPWMSetPwmC(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,CCR);
}

void M1SVPWMSetPwmA(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,CCR);
}
void M1SVPWMSetPwmB(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,CCR);
}
void M1SVPWMSetPwmC(uint32_t CCR){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,CCR);
}
