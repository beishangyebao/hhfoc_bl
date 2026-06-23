/*
 * hhGetIq.c
 *
 *  Created on: Mar 16, 2025
 *      Author: KingPC
 */


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
float GetIqM0(){//获取M0电机的Iq不带滤波
 return hhIqM0.Iq;
}
float GetFilterIqM0(){//获取M0电机的Iq带滤波
  return hhIqM0.Filter_Iq;
}
void IqSensorUpdateM0( float angle_el){//实时更新函数
//	StartAndGetResult();///////////////////////////////////////////////
	UpdatePhaseCurrent(&hhIqM0,angle_el);
}
float GetIqM1(){//获取M1电机的Iq不带滤波
 return hhIqM1.Iq;
}
float GetFilterIqM1(){//获取M1电机的Iq带滤波
  return hhIqM1.Filter_Iq;
}
void IqSensorUpdateM1( float angle_el){//实时更新函数
	AD_Value[0]=AD_ValueT[0];
	AD_Value[1]=AD_ValueT[1];
	AD_Value[2]=AD_ValueT[2];
	AD_Value[3]=AD_ValueT[3];
	UpdatePhaseCurrent(&hhIqM1,angle_el);
}
uint16_t GetAD0(){
	return AD_Value[0];
}
uint16_t GetAD1(){
	return AD_Value[1];
}
uint16_t GetAD2(){
	return AD_Value[2];
}
uint16_t GetAD3(){
	return AD_Value[3];
}

