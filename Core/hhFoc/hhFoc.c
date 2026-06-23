/*
 * hhFoc.c
 *
 *  Created on: Mar 12, 2025
 *      Author: KingPC
 */



#include"hhFoc.h"
#include"math.h"
#include"tim.h"
#include"../hhPwm/hhPwm.h"
#include "../hhSerial/hhSerial.h"
#include "../hhAS5600/hhAS5600.h"
#include "../hhPID/hhPID.h"
#include "../hhGetIq/hhGetIq.h"
#define _3PI_2 4.71238898038f
struct Motor Motor0 = { 0 };
struct Motor Motor1 = { 1 };
float voltage_power_supply;
int PP;
int DIR;
//限制只能输出low~high之间的数值
double _constrain(double amt, double low, double high) {
	if (amt < low) {
		return low;
	} else if (amt > high) {
		return high;
	} else {
		return amt;
	}
}
//限定角度只能是0~360度，不能出现负角度
float _normalizeAngle(float angle) {
	float a = fmod(angle, 2 * M_PI);  //取余保证角度是-360~360度
	if (a >= 0) {
		return a;
	} else {
		return a + 2 * M_PI;
	}
}
//由计算出三相电压设定PWM
void SetPwm(struct Motor *Motor) {
	//限制电压为0~最大设定电压
	Motor->Ua = _constrain(Motor->Ua, 0.0f, voltage_power_supply);
	Motor->Ub = _constrain(Motor->Ub, 0.0f, voltage_power_supply);
	Motor->Uc = _constrain(Motor->Uc, 0.0f, voltage_power_supply);
	//计算占空比并限定占空比的取值范围为0~1
	Motor->dc_a = _constrain(Motor->Ua / voltage_power_supply, 0.0f, 1.0f);
	Motor->dc_b = _constrain(Motor->Ub / voltage_power_supply, 0.0f, 1.0f);
	Motor->dc_c = _constrain(Motor->Uc / voltage_power_supply, 0.0f, 1.0f);
	//写入PWM,详情请看《5、ESP32前置知识.md》
	if (Motor->Mot_num == 0) {
		M0SetPwmA(Motor->dc_a);
		M0SetPwmB(Motor->dc_b);
		M0SetPwmC(Motor->dc_c);
	}else{
		M1SetPwmA(Motor->dc_a);
		M1SetPwmB(Motor->dc_b);
		M1SetPwmC(Motor->dc_c);
	}
}
void setPhaseVoltage(struct Motor *Motor, float Uq, float Ud, float angle_el) {
	Uq = _constrain(Uq, -(voltage_power_supply) / 2,
			(voltage_power_supply) / 2);
	angle_el = _normalizeAngle(angle_el);
	// 帕克逆变换
	Motor->Ualpha = -Uq * sin(angle_el);
	Motor->Ubeta = Uq * cos(angle_el);
	if(Motor->pwmMode==PWM_MODE_SPWM){
		//SPWM
		// 克拉克逆变换
		Motor->Ua = Motor->Ualpha + voltage_power_supply / 2;
		Motor->Ub = (sqrt(3) * Motor->Ubeta - Motor->Ualpha) / 2 + voltage_power_supply / 2;
		Motor->Uc = (-Motor->Ualpha - sqrt(3) * Motor->Ubeta) / 2 + voltage_power_supply / 2;
		  //这里三相的每一相都 + voltage_power_supply/2;作用
		  //1.这里的三相Ua/Ub/Uc是参与Pwm计算的，而PWM在单片机中PWM输出电压输出范围是0V到供电电压Vdc(单片机)，不加voltage_power_supply/2 的话三相都有可能出现负数，导致单片机无法输出PWM(单片机无法输出负电压的PWM)
		  //2.这里还有一个干扰理解的，FOC三相中，在6步换相中，某一相肯定在某个时刻是电流流入(正电压)，某一其他时刻是电流流出(负电压)的，为什么前面说不能有负电压呢？
		  //3.其实并不冲突，首先肯定的是第2点中FOC换相时某一相在不同时刻会有正电压和负电压。这一相的正负电压就是由第1点的PWM(都是正电压没有负电压)产生了的。要理解这点就需要知道3路PWM是如何控制6个MOS管的了
		  //4.1路PWM可以控制2个半桥(上桥臂和下桥臂)
		  //**PWM为高电平时**：上桥臂MOS管导通，下桥臂MOS管关断，将电源电压施加到相应的绕组上。
		  //**PWM为低电平时**：上桥臂MOS管关断，在上桥臂关断后，稍等一段时间(死区时间，防止上下桥臂有短暂的导通造成短路)再让下桥臂导通。
		  //也就是说某一路PWM(都是正电压没有负电压)为高电平时，上桥臂导通，对应Ua线圈电流流入，这时Ua为正电压;PWM(都是正电压没有负电压)为低电平时,上桥臂关断,下桥臂导通。此时有可能在Ua线圈电流流出，这时Ua为负电压。
		SetPwm(Motor);
	}else{
		//SVPWM
		//1.扇区判断
		  int N=0;
		  if ( Motor->Ubeta > 0.0F) {
		    N = 1;
		  }

		  if ((1.73205078F *  Motor->Ualpha -  Motor->Ubeta) / 2.0F > 0.0F) {
		    N += 2;
		  }

		  if ((-1.73205078F *  Motor->Ualpha -  Motor->Ubeta) / 2.0F > 0.0F) {
		    N += 4;
		  }
		  //N=3: 1~60       扇区1
		  //N=1: 60~120     扇区2
		  //N=5: 120~180    扇区3
		  //N=4: 180~240    扇区4
		  //N=6: 180~300    扇区5
		  //N=2: 300~360    扇区6

		 //2.计算作用时间Tx/Ty/Tn
		  float Ts=htim2.Init.Period*2;//定时器的周期时间，也就是定时器的一个周期的计数器的值
		  float	Udc_temp=voltage_power_supply;//母线供电电压
		  float Tx,Ty;
		  switch (N) {
		   case 1://扇区2
		     Tx = (-1.5F *  Motor->Ualpha + 0.866025388F *  Motor->Ubeta) * (Ts / Udc_temp);
		     Ty = (1.5F *  Motor->Ualpha + 0.866025388F *  Motor->Ubeta) * (Ts / Udc_temp);
		     break;
		   case 2://扇区6
		 	Tx = (1.5F *  Motor->Ualpha + 0.866025388F *  Motor->Ubeta) * (Ts / Udc_temp);
		     Ty = -(1.73205078F *  Motor->Ubeta * Ts / Udc_temp);
		     break;
		   case 3://扇区1
		     Tx = -((-1.5F *  Motor->Ualpha + 0.866025388F *  Motor->Ubeta) * (Ts / Udc_temp));
		     Ty = 1.73205078F *  Motor->Ubeta * Ts / Udc_temp;
		     break;
		   case 4://扇区4
		     Tx = -(1.73205078F *  Motor->Ubeta * Ts / Udc_temp);
		     Ty = (-1.5F *  Motor->Ualpha + 0.866025388F *  Motor->Ubeta) * (Ts / Udc_temp);
		     break;
		   case 5://扇区3
		     Tx = 1.73205078F *  Motor->Ubeta * Ts / Udc_temp;
		     Ty = -((1.5F *  Motor->Ualpha + 0.866025388F *  Motor->Ubeta) * (Ts / Udc_temp));
		     break;
		   default://扇区5
		     Tx = -((1.5F *  Motor->Ualpha + 0.866025388F *  Motor->Ubeta) * (Ts / Udc_temp));
		     Ty = -((-1.5F *  Motor->Ualpha + 0.866025388F *  Motor->Ubeta) * (Ts / Udc_temp));
		     break;
		   }
		//3.计算PWM的CCR值
		  float Tcmp1,Tcmp2,Tcmp3,f_temp,Ta,Tb,Tc;
		  f_temp = Tx + Ty;
		  if (f_temp > Ts) {
		    Tx /= f_temp;
		    Ty /= (Tx + Ty);
		  }

		  Ta = (Ts - (Tx + Ty)) / 4.0F;//(Ts - Tx - Ty) / 4.0F
		  Tb = Tx / 2.0F + Ta;                //(Ts + Tx - Ty) / 4.0F
		  Tc = Ty / 2.0F + Tb;                //(Ts + Tx + Ty) / 4.0F
		  switch (N) {
		  case 1://扇区2
		    Tcmp1 = Tb;
		    Tcmp2 = Ta;
		    Tcmp3 = Tc;
		    break;

		  case 2://扇区6
		    Tcmp1 = Ta;
		    Tcmp2 = Tc;
		    Tcmp3 = Tb;
		    break;

		  case 3://扇区1
		    Tcmp1 = Ta;
		    Tcmp2 = Tb;
		    Tcmp3 = Tc;
		    break;

		  case 4://扇区4
		    Tcmp1 = Tc;
		    Tcmp2 = Tb;
		    Tcmp3 = Ta;
		    break;

		  case 5://扇区3
		    Tcmp1 = Tc;
		    Tcmp2 = Ta;
		    Tcmp3 = Tb;
		    break;

		  case 6://扇区5
		    Tcmp1 = Tb;
		    Tcmp2 = Tc;
		    Tcmp3 = Ta;
		    break;
		  default:
			    Tcmp1 = Ts/2;
			    Tcmp2 = Ts/2;
			    Tcmp3 = Ts/2;
			    break;
		  }
		//4.设置CCR值到定时生成PWM波形
	         //第一路PWM波形CCR设置为Tcmp1
	         //第二一路PWM波形CCR设置为Tcmp2
	         //第三一路PWM波形CCR设置为Tcmp3
		  if (Motor->Mot_num == 0) {
		  	  M0SVPWMSetPwmA(Tcmp1);
		  	  M0SVPWMSetPwmB(Tcmp2);
		  	  M0SVPWMSetPwmC(Tcmp3);
		  }else{
		  	  M1SVPWMSetPwmA(Tcmp1);
		  	  M1SVPWMSetPwmB(Tcmp2);
		  	  M1SVPWMSetPwmC(Tcmp3);
		  }


	}
}

float zero_electric_angle_M0 = 0;
float zero_electric_angle_M1 = 0;
float _electricalAngle_M0() {
  return _normalizeAngle((float) PP *getAngle_Without_trackM0() - zero_electric_angle_M0);
}
float _electricalAngle_M1() {
  return _normalizeAngle((float) PP *getAngle_Without_trackM1() - zero_electric_angle_M1);
}
void AlignElectricalAngleM0(){
	  setPhaseVoltage(&Motor0, 3, 0, _3PI_2);
	  HAL_Delay(1000);
	  Sensor_update();
	  zero_electric_angle_M0 = _electricalAngle_M0();
	  setPhaseVoltage(&Motor0, 0, 0, _3PI_2);
	  HAL_Delay(1000);
	  Serial_Printf("%s\r\n", "0电角度：",getAngleM0());
}
void AlignElectricalAngleM1(){
	  setPhaseVoltage(&Motor1, 3, 0, _3PI_2);
	  HAL_Delay(1000);
	  Sensor_update();
	  zero_electric_angle_M1 = _electricalAngle_M1();
	  setPhaseVoltage(&Motor1, 0, 0, _3PI_2);
	  HAL_Delay(1000);
	  Serial_Printf("%s\r\n", "1电角度：",getAngleM1());
}

void AlignElectricalTest0() {  //闭环位置
	setPhaseVoltage(&Motor0, 3, 0, _electricalAngle_M0());
}
void AlignElectricalTest1() {  //闭环位置
	setPhaseVoltage(&Motor1, 3, 0, _electricalAngle_M1());
}
HAL_StatusTypeDef TIM_CounterMode_Reconfigure(TIM_HandleTypeDef *hhTim,uint32_t counterMode)
{
    HAL_StatusTypeDef status = HAL_OK;
    // 修改计数模式
    hhTim->Init.CounterMode = counterMode;
    // 重新初始化定时器基础配置
    status = HAL_TIM_Base_Init(&htim2);
    if(status != HAL_OK)
    {
        return status;
    }
    return status;
}
HAL_StatusTypeDef TIM_OCMode_Reconfigure(TIM_HandleTypeDef *hhTim, uint32_t channel, uint32_t ocMode)
{
    HAL_StatusTypeDef status = HAL_OK;
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = ocMode;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(hhTim, &sConfigOC, channel) != HAL_OK)
    {
    	return status;
    }
    return status;
}

void hhChangeSpwmModel(){
	Motor0. pwmMode=PWM_MODE_SPWM;//切换电机0为SPWM模式
	//设置电机0对应的定时器为向上计数模式且输出PWM模式为TIM_OCMODE_PWM1
	TIM_CounterMode_Reconfigure(&htim2,TIM_COUNTERMODE_UP);
	TIM_OCMode_Reconfigure(&htim2,TIM_CHANNEL_1,TIM_OCMODE_PWM1);
	TIM_OCMode_Reconfigure(&htim2,TIM_CHANNEL_2,TIM_OCMODE_PWM1);
	TIM_OCMode_Reconfigure(&htim2,TIM_CHANNEL_3,TIM_OCMODE_PWM1);

	Motor1. pwmMode=PWM_MODE_SPWM;//切换电机1为SPWM模式
	//设置电机1对应的定时器为向上计数模式且输出PWM模式为TIM_OCMODE_PWM1
	TIM_CounterMode_Reconfigure(&htim2,TIM_COUNTERMODE_UP);
	TIM_OCMode_Reconfigure(&htim2,TIM_CHANNEL_4,TIM_OCMODE_PWM1);
	TIM_OCMode_Reconfigure(&htim3,TIM_CHANNEL_1,TIM_OCMODE_PWM1);
	TIM_OCMode_Reconfigure(&htim3,TIM_CHANNEL_2,TIM_OCMODE_PWM1);
}
void hhChangeSVpwmModel(){
	Motor0. pwmMode=PWM_MODE_SVPWM;//切换电机0为SVPWM模式
	//设置电机0对应的定时器为中心对齐模式且输出PWM模式为TIM_OCMODE_PWM2
	TIM_CounterMode_Reconfigure(&htim2,TIM_COUNTERMODE_CENTERALIGNED1);
	TIM_OCMode_Reconfigure(&htim2,TIM_CHANNEL_1,TIM_OCMODE_PWM2);
	TIM_OCMode_Reconfigure(&htim2,TIM_CHANNEL_2,TIM_OCMODE_PWM2);
	TIM_OCMode_Reconfigure(&htim2,TIM_CHANNEL_3,TIM_OCMODE_PWM2);


	Motor1. pwmMode=PWM_MODE_SVPWM;//切换电机1为SVPWM模式
	//设置电机1对应的定时器为向上计数模式且输出PWM模式为TIM_OCMODE_PWM2
	TIM_CounterMode_Reconfigure(&htim3,TIM_COUNTERMODE_CENTERALIGNED1);
	TIM_OCMode_Reconfigure(&htim4,TIM_CHANNEL_1,TIM_OCMODE_PWM2);
	TIM_OCMode_Reconfigure(&htim3,TIM_CHANNEL_1,TIM_OCMODE_PWM2);
	TIM_OCMode_Reconfigure(&htim3,TIM_CHANNEL_2,TIM_OCMODE_PWM2);
}

void hhFocInit(float power_supply, int _PP, int _DIR,pwm_mode_t pwmMode) {
	if(pwmMode==PWM_MODE_SPWM){
		hhChangeSpwmModel();
	}else{
		hhChangeSVpwmModel();
	}
	PP = _PP;
	DIR = _DIR;
	voltage_power_supply = power_supply;
	InitPwm();
	StartSerialITReceive();
	MyI2C_Init();
	HAL_TIM_Base_Start_IT(&htim4);
	AS5600M0.DIR=DIR;
	AS5600M0.hhLowPassFilterVel.Tf=0.01;
	AS5600M1.DIR=DIR;
	AS5600M1.hhLowPassFilterVel.Tf=0.01;
	AlignElectricalAngleM0();
	AlignElectricalAngleM1();
	InitGetIq();
	Serial_Printf("%s\r\n", "完成PWM初始化设置");
}
void SetsetPhaseVoltageM0(float Uq, float Ud, float angle_el) {
	setPhaseVoltage(&Motor0, Uq, Ud, angle_el);
}
void SetsetPhaseVoltageM1(float Uq, float Ud, float angle_el) {
	setPhaseVoltage(&Motor1, Uq, Ud, angle_el);
}
float OpenLoopShaftAngle=0;
void hhOpenLoopVelM0(float target_velocity){
  //计算当前每个Loop的运行时间间隔
  float Ts = 1 * 1e-3f;
  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  OpenLoopShaftAngle = _normalizeAngle(OpenLoopShaftAngle + target_velocity*Ts);
  float Uq = voltage_power_supply/3;
  SetsetPhaseVoltageM0(Uq,  0, PP * OpenLoopShaftAngle);
}
float OpenLoopShaftAngle1=0;
void hhOpenLoopVelM1(float target_velocity){
  //计算当前每个Loop的运行时间间隔
  float Ts = 1 * 1e-3f;
  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  OpenLoopShaftAngle1 = _normalizeAngle(OpenLoopShaftAngle1 + target_velocity*Ts);
  float Uq = voltage_power_supply/3;
  SetsetPhaseVoltageM1(Uq,  0, PP * OpenLoopShaftAngle1);
}
////////////////////////////////////////////////////////
////////////////闭环位置//////////////////////////////////
void hhCloseLoopPos(float Target) {  //闭环位置
		setPhaseVoltage(&Motor0, _constrain(0.133 * (Target-getAngleM0())*180/M_PI,-6,6), 0, _electricalAngle_M0());
}
void hhCloseLoopPos1(float Target) {  //闭环位置
		setPhaseVoltage(&Motor1, _constrain(0.133 * (Target-getAngleM1())*180/M_PI,-6,6), 0, _electricalAngle_M1());
}
////////////////////////////////////////////////////////
////////////////闭环速度PID//////////////////////////////////
struct hhPID hhClosLoopVelPID_Obj_M0 ;
void hhSetCloseLoopVel_PIDM0(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopVelPID_Obj_M0.P = mP;
	hhClosLoopVelPID_Obj_M0.I = mI;
	hhClosLoopVelPID_Obj_M0.D = mD;
	hhClosLoopVelPID_Obj_M0.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopVelPID_Obj_M0.limit = mlimit;  // PID控制器输出限幅
}
void hhCloseLoopVelM0(float Target) {  //闭环速度
  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopVelPID_Obj_M0,(Target - GetFilterV_M0()) * 180 / M_PI), 0, _electricalAngle_M0());
}
struct hhPID hhClosLoopVelPID_Obj_M1 ;
void hhSetCloseLoopVel_PIDM1(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopVelPID_Obj_M1.P = mP;
	hhClosLoopVelPID_Obj_M1.I = mI;
	hhClosLoopVelPID_Obj_M1.D = mD;
	hhClosLoopVelPID_Obj_M1.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopVelPID_Obj_M1.limit = mlimit;  // PID控制器输出限幅
}
void hhCloseLoopVelM1(float Target) {  //闭环速度
  setPhaseVoltage(&Motor1, PidRun(&hhClosLoopVelPID_Obj_M1,(Target - GetFilterV_M1()) * 180 / M_PI), 0, _electricalAngle_M1());
}
////////////////////////////////////////////////////////
////////////////闭环位置PID//////////////////////////////////
struct hhPID hhClosLoopPosPID_Obj_M0 ;
void hhSetCloseLoopPos_PIDM0(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopPosPID_Obj_M0.P = mP;
	hhClosLoopPosPID_Obj_M0.I = mI;
	hhClosLoopPosPID_Obj_M0.D = mD;
	hhClosLoopPosPID_Obj_M0.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopPosPID_Obj_M0.limit = mlimit;  // PID控制器输出限幅
}
void hhCloseLoopPosM0(float Target) {  //闭环速度
  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/M_PI), 0, _electricalAngle_M0());
}

struct hhPID hhClosLoopPosPID_Obj_M1 ;
void hhSetCloseLoopPos_PIDM1(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopPosPID_Obj_M1.P = mP;
	hhClosLoopPosPID_Obj_M1.I = mI;
	hhClosLoopPosPID_Obj_M1.D = mD;
	hhClosLoopPosPID_Obj_M1.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopPosPID_Obj_M1.limit = mlimit;  // PID控制器输出限幅
}
void hhCloseLoopPosM1(float Target) {  //闭环速度
  setPhaseVoltage(&Motor1, PidRun(&hhClosLoopPosPID_Obj_M1,(Target-getAngleM1())*180/M_PI), 0, _electricalAngle_M1());
}
////////////////////////////////////////////////////////
////////////////刷新传感器函数//////////////////////////////////
void UpdateAllSensor(){
	Sensor_update();//刷新AS5600
	IqSensorUpdateM1(_electricalAngle_M1());//电机1 Iq电流获取
	IqSensorUpdateM0(_electricalAngle_M0());//电机0 Iq电流获取
}
////////////////////////////////////////////////////////
////////////////闭环Iq PID//////////////////////////////////
struct hhPID hhClosLoopIqPID_Obj_M0 ;
void hhSetCloseLoopIq_PIDM0(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopIqPID_Obj_M0.P = mP;
	hhClosLoopIqPID_Obj_M0.I = mI;
	hhClosLoopIqPID_Obj_M0.D = mD;
	hhClosLoopIqPID_Obj_M0.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopIqPID_Obj_M0.limit = mlimit;  // PID控制器输出限幅
}
void hhCloseLoopIqM0(float Target) {  //闭环速度
  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopIqPID_Obj_M0,Target - GetFilterIqM0() ), 0, _electricalAngle_M0());
}

struct hhPID hhClosLoopIqPID_Obj_M1 ;
void hhSetCloseLoopIq_PIDM1(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopIqPID_Obj_M1.P = mP;
	hhClosLoopIqPID_Obj_M1.I = mI;
	hhClosLoopIqPID_Obj_M1.D = mD;
	hhClosLoopIqPID_Obj_M1.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopIqPID_Obj_M1.limit = mlimit;  // PID控制器输出限幅
}
void hhCloseLoopIqM1(float Target) {  //闭环速度
  setPhaseVoltage(&Motor1, PidRun(&hhClosLoopIqPID_Obj_M1,Target - GetFilterIqM1() ), 0, _electricalAngle_M1());
}
////////////////////////////////////////////////////////
////////////////闭环速度 闭环Iq PID//////////////////////////////////
void hhCloseLoopVel_WithIqM0(float Target) {  //带电流环的闭环速度
  // setPhaseVoltage(&Motor0, PidRun(&hhClosLoopVelPID_Obj_M0,(Target - GetFilterV_M0()) * 180 / M_PI), 0, _electricalAngle_M0());//改进前
  hhCloseLoopIqM0(PidRun(&hhClosLoopVelPID_Obj_M0,(Target - GetFilterV_M0()) * 180 / M_PI)) ;//改进后
}
void hhCloseLoopVel_WithIqM1(float Target) {  //带电流环的闭环速度
  // setPhaseVoltage(&Motor0, PidRun(&hhClosLoopVelPID_Obj_M0,(Target - GetFilterV_M0()) * 180 / M_PI), 0, _electricalAngle_M0());//改进前
  hhCloseLoopIqM1(PidRun(&hhClosLoopVelPID_Obj_M1,(Target - GetFilterV_M1()) * 180 / M_PI)) ;//改进后
}
////////////////////////////////////////////////////////
////////////////闭环位置 闭环Iq PID//////////////////////////////////
void hhCloseLoopPos_WithIqM0(float Target) {  //带电流环的闭环位置
  //  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/M_PI), 0, _electricalAngle_M0());//改进前
	hhCloseLoopIqM0(PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/M_PI));  //改进后
}
void hhCloseLoopPos_WithIqM1(float Target) {  //带电流环的闭环位置
  //  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/M_PI), 0, _electricalAngle_M0());//改进前
	hhCloseLoopIqM1(PidRun(&hhClosLoopPosPID_Obj_M1,(Target-getAngleM1())*180/M_PI));  //改进后
}
////////////////////////////////////////////////////////
////////////////闭环位置 闭环速度 闭环Iq PID//////////////////////////////////
void hhCloseLoopPos_WithVelIqM0(float Target) {
	hhCloseLoopIqM0(PidRun(&hhClosLoopVelPID_Obj_M0,PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/M_PI)-GetFilterV_M0()));
}
void hhCloseLoopPos_WithVelIqM1(float Target) {
	hhCloseLoopIqM1(PidRun(&hhClosLoopVelPID_Obj_M1,PidRun(&hhClosLoopPosPID_Obj_M1,(Target-getAngleM1())*180/M_PI)-GetFilterV_M1()));
}






