

#include"adc.h"
#include"hhGetIq.h"
#include <math.h>
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define _ADC_VOLTAGE 3.3f        //ADC 电压
#define _ADC_RESOLUTION 4095.0f  //ADC 分辨率
#define AmpGain 50.0f            //传感器放大倍率
uint16_t AD_Value[4];
uint16_t AD_ValueT[4];
struct hhIq hhIqM0 = { .Mot_num = 0 };
struct hhIq hhIqM1 = { .Mot_num = 1 };
float _shunt_resistor;
int CountOutPutT = 0;

/**
 * @brief 初始化电流采样链路。
 *
 * 所在流程：
 * `hhFocInit()` 在 PWM、串口、AS5600、电角度对齐之后调用本函数。
 * 初始化完成后，`UpdateAllSensor()` 每 1 ms 调用 `IqSensorUpdateM0/M1()`
 * 刷新两台电机的 Iq，供电流环 PID 使用。
 *
 * 初始化步骤：
 * 1. 校准 ADC1 外设；
 * 2. 启动 ADC1 + DMA 连续扫描 4 个通道，采样结果进入 `AD_ValueT`；
 * 3. 设置采样电阻阻值；
 * 4. 对 M0/M1 分别做零电流偏置校准；
 * 5. 设置两台电机 Iq 低通滤波时间常数。
 *
 * @note 校准期间电机最好不通电/不转动，否则零点偏置会被实际电流污染。
 */
void InitGetIq() {
	HAL_ADCEx_Calibration_Start(&hadc1);//校准ADC外设
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) AD_ValueT, 4);//打开ADC_DMA
	_shunt_resistor = 0.01;
	//校准0点
	calibrateOffsets(&hhIqM0);
	calibrateOffsets(&hhIqM1);
	hhIqM0.hhLowPassFilterIq.Tf = 0.05;
	hhIqM1.hhLowPassFilterIq.Tf = 0.05;
}

/**
 * @brief 采集多次 ADC 原始值，计算某台电机两相电流的零点偏置。
 *
 * @param hhIqP 指向 M0 或 M1 的电流状态对象。
 *
 * 通道关系：
 * - M0 使用 `AD_ValueT[0]`、`AD_ValueT[1]`；
 * - M1 使用 `AD_ValueT[2]`、`AD_ValueT[3]`。
 *
 * 流程：
 * 1. 清零 A/B 相偏置累加器；
 * 2. 连续读取 1000 次 DMA 正在更新的 ADC 值；
 * 3. 每次间隔 1 ms，降低瞬时噪声影响；
 * 4. 求平均，作为后续电流换算时要扣除的零点。
 *
 * @note 这里得到的是 ADC 计数值偏置，不是伏特或安培。
 */
void calibrateOffsets(struct hhIq *hhIqP) {//校准0点误差
	const int calibration_rounds = 1000;
	hhIqP->offset_ia = 0;
	hhIqP->offset_ib = 0;
	for (int i = 0; i < calibration_rounds; i++) {
		if(hhIqP->Mot_num==0){
			hhIqP->offset_ia += AD_ValueT[0];
			hhIqP->offset_ib += AD_ValueT[1];
		}else{
			hhIqP->offset_ia += AD_ValueT[2];
			hhIqP->offset_ib += AD_ValueT[3];
		}
		HAL_Delay(1);
	}
	// 求平均，得到误差
	hhIqP->offset_ia = hhIqP->offset_ia / calibration_rounds;
	hhIqP->offset_ib = hhIqP->offset_ib / calibration_rounds;
}

/**
 * @brief 根据 ADC 采样值和当前电角度计算 q 轴电流 Iq。
 *
 * 所在流程：
 * `UpdateAllSensor()` 每 1 ms 调用 `IqSensorUpdateM0()` 和
 * `IqSensorUpdateM1()`，它们最终进入本函数。本函数输出的
 * `Filter_Iq` 会被 `hhCloseLoopIqM0/M1()` 的电流环 PID 使用。
 *
 * @param hhIqP 指向 M0 或 M1 的电流状态对象。
 * @param angle_el 当前电角度，单位 rad，由 `hhFoc.c` 根据 AS5600 机械角度计算。
 *
 * 计算流程：
 * 1. 按电机编号选择对应的两路 ADC 快照；
 * 2. 扣除零点偏置，得到相对 ADC 计数；
 * 3. 计数值换算为 ADC 电压，再除以 INA240 放大倍数；
 * 4. 除以采样电阻得到 A/B 相电流，单位 A；
 * 5. 做 Clarke 变换：两相电流 -> α/β 静止坐标系；
 * 6. 做 Park 变换：α/β -> d/q 旋转坐标系；
 * 7. 只保存当前控制需要的 q 轴电流 Iq，并做低通滤波。
 *
 * @note 乘以 -1 是为了匹配当前硬件中采样电阻/放大器输入方向。
 */
void UpdatePhaseCurrent(struct hhIq *hhIqP, float angle_el) {//由电角度计算出Iq
	//-1表示原理图中的电阻是反接在INA240A引脚上的
	//(ReadADC(pinA) - offset_ia)：获得INA240A校准后的ADC值
	//(((ReadADC(pinA) - offset_ia) * _ADC_VOLTAGE) / _ADC_RESOLUTION):通过INA240A的参考电压获得INA240A测得的电压值
	//((((ReadADC(pinA) - offset_ia) * _ADC_VOLTAGE) / _ADC_RESOLUTION) / AmpGain):INA240A采集到的数据是放大50倍才通过ADC输出的，所以单片机获得后需要除以50
	//((((ReadADC(pinB) - offset_ib) * _ADC_VOLTAGE) / _ADC_RESOLUTION) / AmpGain) / _shunt_resistor:最终电压除以电阻计算得到电流
	if (hhIqP->Mot_num == 0) {
		hhIqP->current_a = -1 * ((((AD_Value[0] - hhIqP->offset_ia) * _ADC_VOLTAGE) / _ADC_RESOLUTION) / AmpGain) / _shunt_resistor;
		hhIqP->current_b = -1 * ((((AD_Value[1] - hhIqP->offset_ib) * _ADC_VOLTAGE) / _ADC_RESOLUTION) / AmpGain) / _shunt_resistor;
	} else {
//		Serial_Printf("%f,%f\r\n",AD_Value[2],AD_Value[3]);
		hhIqP->current_a = -1 * ((((AD_Value[2] - hhIqP->offset_ia) * _ADC_VOLTAGE) / _ADC_RESOLUTION) / AmpGain) / _shunt_resistor;
		hhIqP->current_b = -1 * ((((AD_Value[3] - hhIqP->offset_ib) * _ADC_VOLTAGE) / _ADC_RESOLUTION) / AmpGain) / _shunt_resistor;

//		Serial_Printf("%d,%d\r\n",AD_Value[2],AD_Value[3]);
	}

	//Clarke变换
	float I_alpha = hhIqP->current_a;
	float I_beta = _1_SQRT3 * hhIqP->current_a + _2_SQRT3 * hhIqP->current_b;
	//Park变换，需要电角度
	float ct = cos(angle_el);
	float st = sin(angle_el);
	//float I_d = I_alpha * ct + I_beta * st;//用不到不计算
	hhIqP->Iq = I_beta * ct - I_alpha * st;
	// 计算时间间隔
	float d_Ts = 1 * 1e-3;	  //1ms
	hhIqP->hhLowPassFilterIq.dt = d_Ts;
	hhIqP->Filter_Iq = hhGetFilterValue(&(hhIqP->hhLowPassFilterIq), hhIqP->Iq);//计算滤波
}

/**
 * @brief 获取 M0 未滤波 Iq。
 * @return M0 q 轴电流，单位 A。
 */
float GetIqM0(){//获取M0电机的Iq不带滤波
 return hhIqM0.Iq;
}

/**
 * @brief 获取 M0 低通滤波后的 Iq。
 * @return M0 滤波 q 轴电流，单位 A。
 */
float GetFilterIqM0(){//获取M0电机的Iq带滤波
  return hhIqM0.Filter_Iq;
}

/**
 * @brief 刷新 M0 的 Iq。
 *
 * @param angle_el M0 当前电角度，单位 rad。
 * @note 当前函数直接使用 `AD_Value` 快照；快照复制动作在 `IqSensorUpdateM1()` 中完成。
 */
void IqSensorUpdateM0( float angle_el){//实时更新函数
//	StartAndGetResult();///////////////////////////////////////////////
	UpdatePhaseCurrent(&hhIqM0,angle_el);
}

/**
 * @brief 获取 M1 未滤波 Iq。
 * @return M1 q 轴电流，单位 A。
 */
float GetIqM1(){//获取M1电机的Iq不带滤波
 return hhIqM1.Iq;
}

/**
 * @brief 获取 M1 低通滤波后的 Iq。
 * @return M1 滤波 q 轴电流，单位 A。
 */
float GetFilterIqM1(){//获取M1电机的Iq带滤波
  return hhIqM1.Filter_Iq;
}

/**
 * @brief 刷新 M1 的 Iq，并把 DMA 实时缓冲复制到控制计算快照。
 *
 * @param angle_el M1 当前电角度，单位 rad。
 *
 * 流程：
 * 1. 将 `AD_ValueT[0..3]` 复制到 `AD_Value[0..3]`；
 * 2. 用复制后的快照计算 M1 的相电流和 Iq。
 *
 * @note 因为 `UpdateAllSensor()` 先调用 M1 再调用 M0，所以 M0 随后也会使用同一份快照。
 *       这种做法能让两台电机尽量基于同一时刻的 ADC 数据计算。
 */
void IqSensorUpdateM1( float angle_el){//实时更新函数
	AD_Value[0]=AD_ValueT[0];
	AD_Value[1]=AD_ValueT[1];
	AD_Value[2]=AD_ValueT[2];
	AD_Value[3]=AD_ValueT[3];
	UpdatePhaseCurrent(&hhIqM1,angle_el);
}

/**
 * @brief 读取 ADC 快照通道 0。
 * @return ADC 原始计数值；当前作为 M0 A 相电流采样。
 */
uint16_t GetAD0(){
	return AD_Value[0];
}

/**
 * @brief 读取 ADC 快照通道 1。
 * @return ADC 原始计数值；当前作为 M0 B 相电流采样。
 */
uint16_t GetAD1(){
	return AD_Value[1];
}

/**
 * @brief 读取 ADC 快照通道 2。
 * @return ADC 原始计数值；当前作为 M1 A 相电流采样。
 */
uint16_t GetAD2(){
	return AD_Value[2];
}

/**
 * @brief 读取 ADC 快照通道 3。
 * @return ADC 原始计数值；当前作为 M1 B 相电流采样。
 */
uint16_t GetAD3(){
	return AD_Value[3];
}

