/*
 * hhAS5600.h
 *
 *  Created on: Mar 13, 2025
 *      Author: KingPC
 */

#ifndef HHAS5600_HHAS5600_H_
#define HHAS5600_HHAS5600_H_
#include"../hhLowPassFilter/hhLowPassFilter.h"

/**
 * @file hhAS5600.h
 * @brief AS5600 磁编码器读取与机械角度/速度估算接口。
 *
 * 本模块负责用 GPIO 模拟 I2C 读取两颗 AS5600 的原始角度，
 * 并把 0~2π 的单圈角度展开成可连续增加/减少的多圈机械角度。
 * `hhFoc.c` 通过这些角度计算电角度、做位置/速度闭环。
 */

/**
 * @brief 单个 AS5600 传感器及其派生机械量。
 *
 * 每个电机对应一个 `struct AS5600`：
 * - M0 使用 `AS5600M0`；
 * - M1 使用 `AS5600M1`。
 *
 * 角度单位均为弧度，速度单位为弧度/秒。
 */
struct AS5600 {
	/** 电机编号：0 表示 M0，非 0 表示 M1；用于选择不同的 SCL/SDA 引脚。 */
	int Mot_num;
	/** 方向系数，通常为 1 或 -1，用于把传感器方向统一到控制方向。 */
	int DIR; //方向
	/** 当前累计整圈数，跨过 0/2π 边界时更新。 */
	int full_rotations_Cur;    //当前次的圈数
	/** 上一次累计整圈数，用于速度差分。 */
	int full_rotations_Pre;    //上一次的圈数
	/** 当前单圈机械角度，范围约为 0~2π，不包含累计圈数。 */
	float angleWithout_track_Cur;  //当前次的不带圈数的角度值
	/** 上一次单圈机械角度，用于判断是否跨圈。 */
	float angleWithout_track_Pre;  //上一次的不带圈数的角度值
	/** 当前多圈机械角度，等于整圈数 * 2π + 单圈角度。 */
	float angle_Cur;               //当前次的带圈数的角度值
	/** 上一次多圈机械角度，用于计算速度。 */
	float angle_Pre;               //上一次的带圈数的角度值
	/** 由相邻两次多圈角度差分得到的机械角速度，单位 rad/s。 */
	float vel;                     //计算出来的速度
	/** 低通滤波后的机械角速度，单位 rad/s。 */
	float Filter_vel;
	/** 速度低通滤波器状态，Tf 在 `hhFocInit()` 中设置。 */
	struct hhLowPassFilter hhLowPassFilterVel;
};

/** M0 的 AS5600 状态对象。 */
extern struct AS5600 AS5600M0;  // 声明外部变量
/** M1 的 AS5600 状态对象。 */
extern struct AS5600 AS5600M1;  // 声明外部变量

/**
 * @brief 同时刷新 M0 和 M1 的角度、速度状态。
 * @note 当前由 `UpdateAllSensor()` 在 1 ms 控制节拍中调用。
 */
void Sensor_update();

/**
 * @brief 初始化模拟 I2C 的 GPIO 空闲电平。
 */
void MyI2C_Init(void) ;

/**
 * @brief 获取 M0 当前单圈机械角度。
 * @return 已乘方向系数的单圈角度，单位 rad，范围约为 -2π~2π。
 */
float getAngle_Without_trackM0();

/**
 * @brief 获取 M0 当前多圈机械角度。
 * @return 已乘方向系数的多圈角度，单位 rad。
 */
float getAngleM0() ;

/**
 * @brief 获取 M0 低通滤波后的机械角速度。
 * @return 已乘方向系数的速度，单位 rad/s。
 */
float GetFilterV_M0();

/**
 * @brief 获取 M0 未滤波机械角速度。
 * @return 已乘方向系数的速度，单位 rad/s。
 */
float GetV_M0();

/**
 * @brief 获取 M1 当前单圈机械角度。
 * @return 已乘方向系数的单圈角度，单位 rad，范围约为 -2π~2π。
 */
float getAngle_Without_trackM1();

/**
 * @brief 获取 M1 当前多圈机械角度。
 * @return 已乘方向系数的多圈角度，单位 rad。
 */
float getAngleM1() ;

/**
 * @brief 获取 M1 低通滤波后的机械角速度。
 * @return 已乘方向系数的速度，单位 rad/s。
 */
float GetFilterV_M1();

/**
 * @brief 获取 M1 未滤波机械角速度。
 * @return 已乘方向系数的速度，单位 rad/s。
 */
float GetV_M1();
#endif /* HHAS5600_HHAS5600_H_ */
