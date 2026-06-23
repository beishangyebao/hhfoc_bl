/*
 * hhAS5600.c
 *
 *  Created on: Mar 13, 2025
 *      Author: KingPC
 */


#include"hhAS5600.h"
#include"gpio.h"
#include <math.h>

#define AS5600_ADDRESS        0x6C
#define AS5600_RAW_ANGLE_H    0x0C
#define AS5600_RAW_ANGLE_L    0x0D
#define PI       3.14159265359f
#define _2PI     6.28318530718f

uint16_t hhSCL_Pin, hhSDA_Pin;
struct AS5600 AS5600M0 = { 0 };
struct AS5600 AS5600M1 = { 1 };
void I2C_W_SCL(GPIO_PinState x) {
	HAL_GPIO_WritePin(GPIOB, hhSCL_Pin, x);
}
//对SDA写
void I2C_W_SDA(GPIO_PinState x) {
	HAL_GPIO_WritePin(GPIOB, hhSDA_Pin, x);
}
//对SDA读
uint8_t I2C_R_SDA(void) {
	uint8_t BitValue;
	if (HAL_GPIO_ReadPin(GPIOB, hhSDA_Pin) == GPIO_PIN_RESET) {
		BitValue = 0;
	} else {
		BitValue = 1;
	}
	return BitValue;
}
void I2C_Start(void) {
	I2C_W_SCL(1);
	I2C_W_SDA(1);
	I2C_W_SDA(0);
	I2C_W_SCL(0);
}

void I2C_Stop(void) {
	I2C_W_SDA(0);
	I2C_W_SCL(1);
	I2C_W_SDA(1);
}

void I2C_SendByte(uint8_t Byte) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		I2C_W_SDA(Byte & (0x80 >> i));
		I2C_W_SCL(1);
		I2C_W_SCL(0);
	}
}

uint8_t I2C_RecviveData(void) {
	uint8_t i, Byte = 0x00;
	I2C_W_SDA(1);
	for (i = 0; i < 8; i++) {
		I2C_W_SCL(1);
		if (I2C_R_SDA() == 1) {
			Byte |= (0x80 >> i);
		}
		I2C_W_SCL(0);
	}
	return Byte;
}

void I2C_SendAck(uint8_t AckBit) {
	I2C_W_SDA(AckBit);
	I2C_W_SCL(1);
	I2C_W_SCL(0);
}

uint8_t I2C_RecviveAck(void) {
	uint8_t AckBit;

	I2C_W_SDA(1);
	I2C_W_SCL(1);
	AckBit = I2C_R_SDA();
	I2C_W_SCL(0);

	return AckBit;
}

//初始化GPIO
void MyI2C_Init(void) {
	//初始时都设置成高电平，因为I2C空闲时两根线都是高电平
	HAL_GPIO_WritePin(GPIOB, M0SCL_Pin | M0SDA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, M1SCL_Pin | M1SDA_Pin, GPIO_PIN_SET);

}
float AS5600_GetRawData(struct AS5600 *AS5600P) {
	if (AS5600P->Mot_num == 0) {
		hhSCL_Pin = M0SCL_Pin;
		hhSDA_Pin = M0SDA_Pin;
	} else {
		hhSCL_Pin = M1SCL_Pin;
		hhSDA_Pin = M1SDA_Pin;
	}
	uint8_t Data_L;
	uint8_t Data_H;
	float Raw_Data = 0;

	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS);
	I2C_RecviveAck();
	I2C_SendByte(AS5600_RAW_ANGLE_H);
	I2C_RecviveAck();

	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	I2C_RecviveAck();
	Data_H = I2C_RecviveData();
	I2C_RecviveAck();

	I2C_Start();
	I2C_SendByte(AS5600_ADDRESS | 0x01);
	I2C_RecviveAck();
	Data_L = I2C_RecviveData();
	I2C_SendAck(1);
	I2C_Stop();

	Raw_Data = (Data_H << 8) | Data_L;

	return Raw_Data;
}
void M0Sensor_update(struct AS5600 *AS5600P) {
	AS5600P->angleWithout_track_Cur =
			(AS5600_GetRawData(AS5600P) / 4096) * _2PI;
	float d_angleWithout_track = AS5600P->angleWithout_track_Cur
			- AS5600P->angleWithout_track_Pre; //传感器获得前后两次数据差值
	// 判断是否发生溢出，并更新全旋转计数
	if (fabs(d_angleWithout_track) > (0.8f * _2PI)) {
		if (d_angleWithout_track > 0) {
			AS5600P->full_rotations_Cur -= 1;
		} else {
			AS5600P->full_rotations_Cur += 1;
		}
	}
	// 计算时间间隔
	float d_Ts = 1 * 1e-3;	  //1ms
	// 计算角速度
	AS5600P->angle_Cur = AS5600P->full_rotations_Cur * _2PI
			+ AS5600P->angleWithout_track_Cur;	  //当前次带圈数的角度
	AS5600P->angle_Pre = AS5600P->full_rotations_Pre * _2PI
			+ AS5600P->angleWithout_track_Pre;	  //上一次带圈数的角度
	AS5600P->vel = (AS5600P->angle_Cur - AS5600P->angle_Pre) / d_Ts;	 //计算速度

	AS5600P->hhLowPassFilterVel.dt=d_Ts;
	AS5600P->Filter_vel=hhGetFilterValue(&(AS5600P->hhLowPassFilterVel),AS5600P->vel);
	// 更新变量
	AS5600P->angleWithout_track_Pre = AS5600P->angleWithout_track_Cur;
	AS5600P->full_rotations_Pre = AS5600P->full_rotations_Cur;
}
void Sensor_update() {
	M0Sensor_update(&AS5600M0);
	M0Sensor_update(&AS5600M1);
}
float getAngle_Without_trackM0() {
  return AS5600M0.DIR* AS5600M0.angleWithout_track_Cur;
}
float getAngleM0() {
  return AS5600M0.DIR* AS5600M0.angle_Cur;
}
float GetFilterV_M0() {
  return AS5600M0.DIR* AS5600M0.Filter_vel;
}
float GetV_M0() {
  return AS5600M0.DIR* AS5600M0.vel;
}


float getAngle_Without_trackM1() {
  return AS5600M1.DIR* AS5600M1.angleWithout_track_Cur;
}
float getAngleM1() {
  return AS5600M1.DIR* AS5600M1.angle_Cur;
}
float GetFilterV_M1() {
  return AS5600M1.DIR* AS5600M1.Filter_vel;
}
float GetV_M1() {
  return AS5600M1.DIR* AS5600M1.vel;
}
