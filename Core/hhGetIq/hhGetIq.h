/*
 * hhGetIq.h
 *
 *  Created on: Mar 16, 2025
 *      Author: KingPC
 */

#ifndef HHGETIQ_HHGETIQ_H_
#define HHGETIQ_HHGETIQ_H_

#include"../hhLowPassFilter/hhLowPassFilter.h"
struct hhIq {
	  int Mot_num;
	  float _shunt_resistor;  //采样电阻阻值
	  float offset_ia;        //A相0电流(点击未转动)时的电压
	  float offset_ib;        //B相0电流(点击未转动)时的电压
	  //用于滤波
	  struct hhLowPassFilter hhLowPassFilterIq;
	  float current_a, current_b;  //最终A/B相电流结果
	  float Iq ;
	  float Filter_Iq;
};
extern struct hhIq hhIqM0;  // 声明外部变量
extern struct hhIq hhIqM1;  // 声明外部变量
void calibrateOffsets(struct hhIq *hhIqP);
void UpdatePhaseCurrent(struct hhIq *hhIqP,float angle_el) ;
void InitGetIq();
float GetIqM0();
float GetFilterIqM0();
void IqSensorUpdateM0( float angle_el);
float GetIqM1();
float GetFilterIqM1();
void IqSensorUpdateM1( float angle_el);
uint16_t GetAD0();
uint16_t GetAD1();
uint16_t GetAD2();
uint16_t GetAD3();
#endif /* HHGETIQ_HHGETIQ_H_ */
