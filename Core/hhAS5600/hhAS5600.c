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

/**
 * @brief 写当前被选中的模拟 I2C SCL 引脚电平。
 *
 * 所在流程：
 * `AS5600_GetRawData()` 会先根据 M0/M1 选择 `hhSCL_Pin`，
 * 后续所有 I2C 时序函数都通过本函数拉高或拉低 SCL。
 *
 * @param x GPIO_PIN_SET 或 GPIO_PIN_RESET。
 */
void I2C_W_SCL(GPIO_PinState x) {
	HAL_GPIO_WritePin(GPIOB, hhSCL_Pin, x);
}
//对SDA写

/**
 * @brief 写当前被选中的模拟 I2C SDA 引脚电平。
 *
 * @param x GPIO_PIN_SET 或 GPIO_PIN_RESET。
 * @note SDA 配置为开漏输出，写 1 表示释放总线，写 0 表示主动拉低。
 */
void I2C_W_SDA(GPIO_PinState x) {
	HAL_GPIO_WritePin(GPIOB, hhSDA_Pin, x);
}
//对SDA读

/**
 * @brief 读取当前被选中的模拟 I2C SDA 引脚电平。
 *
 * @return 0 表示低电平，1 表示高电平。
 * @note 读取 ACK 和接收数据位时使用；GPIO 仍复用同一个 SDA 引脚。
 */
uint8_t I2C_R_SDA(void) {
	uint8_t BitValue;
	if (HAL_GPIO_ReadPin(GPIOB, hhSDA_Pin) == GPIO_PIN_RESET) {
		BitValue = 0;
	} else {
		BitValue = 1;
	}
	return BitValue;
}

/**
 * @brief 产生模拟 I2C 起始条件。
 *
 * 流程：
 * 1. 空闲状态 SCL/SDA 都为高；
 * 2. SCL 保持高时 SDA 从高变低，表示 START；
 * 3. 拉低 SCL，准备传输第一个数据位。
 */
void I2C_Start(void) {
	I2C_W_SCL(1);
	I2C_W_SDA(1);
	I2C_W_SDA(0);
	I2C_W_SCL(0);
}

/**
 * @brief 产生模拟 I2C 停止条件。
 *
 * 流程：
 * 1. SDA 先拉低；
 * 2. SCL 拉高；
 * 3. SCL 为高时 SDA 从低变高，表示 STOP，总线回到空闲。
 */
void I2C_Stop(void) {
	I2C_W_SDA(0);
	I2C_W_SCL(1);
	I2C_W_SDA(1);
}

/**
 * @brief 通过模拟 I2C 发送 1 个字节。
 *
 * @param Byte 要发送的数据，最高位先发。
 * @note 每一位都在 SCL 低电平时设置 SDA，在 SCL 高电平期间由从机采样。
 */
void I2C_SendByte(uint8_t Byte) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		I2C_W_SDA(Byte & (0x80 >> i));
		I2C_W_SCL(1);
		I2C_W_SCL(0);
	}
}

/**
 * @brief 通过模拟 I2C 接收 1 个字节。
 *
 * @return 从 SDA 线上读取的 8 位数据，最高位先接收。
 * @note 接收前释放 SDA，随后每次拉高 SCL 采样一位。
 */
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

/**
 * @brief 主机向从机发送 ACK/NACK。
 *
 * @param AckBit 0 表示 ACK，继续读取；1 表示 NACK，通常表示最后一个字节。
 */
void I2C_SendAck(uint8_t AckBit) {
	I2C_W_SDA(AckBit);
	I2C_W_SCL(1);
	I2C_W_SCL(0);
}

/**
 * @brief 主机读取从机返回的 ACK 位。
 *
 * @return 0 表示从机应答 ACK，1 表示未应答 NACK。
 * @note 当前调用方没有根据返回值做错误处理，只完成时序读取。
 */
uint8_t I2C_RecviveAck(void) {
	uint8_t AckBit;

	I2C_W_SDA(1);
	I2C_W_SCL(1);
	AckBit = I2C_R_SDA();
	I2C_W_SCL(0);

	return AckBit;
}

//初始化GPIO
/**
 * @brief 初始化两路 AS5600 模拟 I2C 总线的空闲电平。
 *
 * 两颗 AS5600 分别使用不同的 SCL/SDA：
 * - M0：`M0SCL_Pin` / `M0SDA_Pin`；
 * - M1：`M1SCL_Pin` / `M1SDA_Pin`。
 *
 * @note GPIO 的开漏模式在 `MX_GPIO_Init()` 中配置；这里仅把总线释放为高电平。
 */
void MyI2C_Init(void) {
	//初始时都设置成高电平，因为I2C空闲时两根线都是高电平
	HAL_GPIO_WritePin(GPIOB, M0SCL_Pin | M0SDA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, M1SCL_Pin | M1SDA_Pin, GPIO_PIN_SET);

}

/**
 * @brief 读取指定 AS5600 的 12 位原始角度值。
 *
 * 所在流程：
 * `M0Sensor_update()` 调用本函数拿到 0~4095 的原始角度，
 * 然后换算成 0~2π 的单圈机械角度。
 *
 * @param AS5600P 指向 M0 或 M1 的 AS5600 状态对象，用于选择对应的模拟 I2C 引脚。
 * @return AS5600 原始角度计数值，理论范围 0~4095。
 *
 * I2C 读取流程：
 * 1. 根据电机编号选择当前操作的 SCL/SDA；
 * 2. 向 AS5600 写入 RAW_ANGLE_H 寄存器地址；
 * 3. 重新起始并读高字节；
 * 4. 再次读取低字节；
 * 5. 合成为 16 位数，实际有效角度为低 12 位。
 *
 * @note 当前代码使用的器件地址为 `0x6C`，与左移后的 7 位地址写法一致。
 */
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

/**
 * @brief 刷新一个 AS5600 对象的角度、累计圈数、速度和滤波速度。
 *
 * 所在流程：
 * `Sensor_update()` 对 M0/M1 各调用一次；`UpdateAllSensor()` 在 1 ms 控制节拍中
 * 调用 `Sensor_update()`，因此本函数默认按 1 ms 间隔计算速度。
 *
 * @param AS5600P 指向要更新的 AS5600 状态对象。
 *
 * 处理流程：
 * 1. 读取 AS5600 原始值并换算成 0~2π 单圈角度；
 * 2. 比较当前单圈角度与上一次单圈角度的差；
 * 3. 如果差值接近一整圈，判断发生 0/2π 跨界并修正累计圈数；
 * 4. 用“累计圈数 + 单圈角度”得到连续多圈机械角度；
 * 5. 用相邻两次多圈角度差除以 1 ms 得到机械速度；
 * 6. 对速度做低通滤波，供速度环使用；
 * 7. 保存当前值作为下次差分的历史值。
 */
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

/**
 * @brief 同步刷新 M0 和 M1 的 AS5600 角度/速度状态。
 *
 * 所在流程：
 * `hhFoc.c` 的 `UpdateAllSensor()` 先调用本函数刷新机械量，
 * 然后根据刷新后的角度计算电角度和 Iq。
 */
void Sensor_update() {
	M0Sensor_update(&AS5600M0);
	M0Sensor_update(&AS5600M1);
}

/**
 * @brief 获取 M0 单圈机械角度。
 * @return 已按 `DIR` 修正方向的单圈角度，单位 rad。
 */
float getAngle_Without_trackM0() {
  return AS5600M0.DIR* AS5600M0.angleWithout_track_Cur;
}

/**
 * @brief 获取 M0 多圈机械角度。
 * @return 已按 `DIR` 修正方向的多圈角度，单位 rad。
 */
float getAngleM0() {
  return AS5600M0.DIR* AS5600M0.angle_Cur;
}

/**
 * @brief 获取 M0 低通滤波后的机械角速度。
 * @return 已按 `DIR` 修正方向的速度，单位 rad/s。
 */
float GetFilterV_M0() {
  return AS5600M0.DIR* AS5600M0.Filter_vel;
}

/**
 * @brief 获取 M0 未滤波机械角速度。
 * @return 已按 `DIR` 修正方向的速度，单位 rad/s。
 */
float GetV_M0() {
  return AS5600M0.DIR* AS5600M0.vel;
}

/**
 * @brief 获取 M1 单圈机械角度。
 * @return 已按 `DIR` 修正方向的单圈角度，单位 rad。
 */
float getAngle_Without_trackM1() {
  return AS5600M1.DIR* AS5600M1.angleWithout_track_Cur;
}

/**
 * @brief 获取 M1 多圈机械角度。
 * @return 已按 `DIR` 修正方向的多圈角度，单位 rad。
 */
float getAngleM1() {
  return AS5600M1.DIR* AS5600M1.angle_Cur;
}

/**
 * @brief 获取 M1 低通滤波后的机械角速度。
 * @return 已按 `DIR` 修正方向的速度，单位 rad/s。
 */
float GetFilterV_M1() {
  return AS5600M1.DIR* AS5600M1.Filter_vel;
}

/**
 * @brief 获取 M1 未滤波机械角速度。
 * @return 已按 `DIR` 修正方向的速度，单位 rad/s。
 */
float GetV_M1() {
  return AS5600M1.DIR* AS5600M1.vel;
}
