/*
 * hhAS5600.h
 *
 *  Created on: Mar 13, 2025
 *      Author: KingPC
 */

#ifndef HHAS5600_HHAS5600_H_
#define HHAS5600_HHAS5600_H_
#include"../hhLowPassFilter/hhLowPassFilter.h"
struct AS5600 {
	int Mot_num;
	int DIR; //方向
	int full_rotations_Cur;    //当前次的圈数
	int full_rotations_Pre;    //上一次的圈数
	float angleWithout_track_Cur;  //当前次的不带圈数的角度值
	float angleWithout_track_Pre;  //上一次的不带圈数的角度值
	float angle_Cur;               //当前次的带圈数的角度值
	float angle_Pre;               //上一次的带圈数的角度值
	float vel;                     //计算出来的速度
	float Filter_vel;
	struct hhLowPassFilter hhLowPassFilterVel;
};
extern struct AS5600 AS5600M0;  // 声明外部变量
extern struct AS5600 AS5600M1;  // 声明外部变量
void Sensor_update();
void MyI2C_Init(void) ;
float getAngle_Without_trackM0();
float getAngleM0() ;
float GetFilterV_M0();
float GetV_M0();
float getAngle_Without_trackM1();
float getAngleM1() ;
float GetFilterV_M1();
float GetV_M1();
#endif /* HHAS5600_HHAS5600_H_ */
