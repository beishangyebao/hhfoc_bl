/*
 * hhGetIq.h
 *
 *  Created on: Mar 16, 2025
 *      Author: KingPC
 */

#ifndef HHGETIQ_HHGETIQ_H_
#define HHGETIQ_HHGETIQ_H_

#include"../hhLowPassFilter/hhLowPassFilter.h"

/**
 * @file hhGetIq.h
 * @brief 两台电机的相电流采样与 q 轴电流 Iq 计算接口。
 *
 * 本模块从 ADC1 + DMA 的四路采样值中取出两台电机的两相电流，
 * 经过零点偏置扣除、电压到电流换算、Clarke 变换和 Park 变换，
 * 得到 FOC 电流环使用的 q 轴电流 `Iq`。
 */

/**
 * @brief 单台电机的电流采样状态。
 *
 * 电流采样只测 A/B 两相，C 相由三相电流和为 0 的关系隐含。
 * `UpdatePhaseCurrent()` 会把 A/B 相电流先转换到 α/β 坐标系，
 * 再结合电角度转换到 d/q 坐标系；当前控制只使用 `Iq`。
 */
struct hhIq {
	  /** 电机编号：0 表示 M0，非 0 表示 M1，用于选择 ADC 通道组。 */
	  int Mot_num;
	  /** 采样电阻阻值，单位 Ω；当前实际使用全局 `_shunt_resistor`。 */
	  float _shunt_resistor;  //采样电阻阻值
	  /** A 相 0 电流时的 ADC 偏置计数平均值。 */
	  float offset_ia;        //A相0电流(点击未转动)时的电压
	  /** B 相 0 电流时的 ADC 偏置计数平均值。 */
	  float offset_ib;        //B相0电流(点击未转动)时的电压
	  //用于滤波
	  /** Iq 低通滤波器状态。 */
	  struct hhLowPassFilter hhLowPassFilterIq;
	  /** 换算后的 A/B 相电流，单位 A。 */
	  float current_a, current_b;  //最终A/B相电流结果
	  /** 未滤波 q 轴电流，单位 A。 */
	  float Iq ;
	  /** 低通滤波后的 q 轴电流，单位 A。 */
	  float Filter_Iq;
};

/** M0 电流采样状态对象。 */
extern struct hhIq hhIqM0;  // 声明外部变量
/** M1 电流采样状态对象。 */
extern struct hhIq hhIqM1;  // 声明外部变量

/**
 * @brief 对指定电机的两路相电流 ADC 做零点偏置校准。
 * @param hhIqP 指向 M0 或 M1 的电流状态对象。
 */
void calibrateOffsets(struct hhIq *hhIqP);

/**
 * @brief 读取 ADC 采样值并计算指定电机的 Iq。
 * @param hhIqP 指向 M0 或 M1 的电流状态对象。
 * @param angle_el 当前电角度，单位 rad。
 */
void UpdatePhaseCurrent(struct hhIq *hhIqP,float angle_el) ;

/**
 * @brief 初始化 ADC 电流采样、零点校准和 Iq 滤波器参数。
 */
void InitGetIq();

/**
 * @brief 获取 M0 未滤波 Iq。
 * @return q 轴电流，单位 A。
 */
float GetIqM0();

/**
 * @brief 获取 M0 滤波后 Iq。
 * @return 低通滤波后的 q 轴电流，单位 A。
 */
float GetFilterIqM0();

/**
 * @brief 用当前电角度刷新 M0 的 Iq。
 * @param angle_el M0 当前电角度，单位 rad。
 */
void IqSensorUpdateM0( float angle_el);

/**
 * @brief 获取 M1 未滤波 Iq。
 * @return q 轴电流，单位 A。
 */
float GetIqM1();

/**
 * @brief 获取 M1 滤波后 Iq。
 * @return 低通滤波后的 q 轴电流，单位 A。
 */
float GetFilterIqM1();

/**
 * @brief 用当前电角度刷新 M1 的 Iq，并同步 DMA 采样快照。
 * @param angle_el M1 当前电角度，单位 rad。
 */
void IqSensorUpdateM1( float angle_el);

/**
 * @brief 获取 ADC 快照通道 0。
 * @return ADC 原始计数值。
 */
uint16_t GetAD0();

/**
 * @brief 获取 ADC 快照通道 1。
 * @return ADC 原始计数值。
 */
uint16_t GetAD1();

/**
 * @brief 获取 ADC 快照通道 2。
 * @return ADC 原始计数值。
 */
uint16_t GetAD2();

/**
 * @brief 获取 ADC 快照通道 3。
 * @return ADC 原始计数值。
 */
uint16_t GetAD3();
#endif /* HHGETIQ_HHGETIQ_H_ */
