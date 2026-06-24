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
#define _PI    3.14159265358f

/**
 * @file hhFoc.c
 * @brief FOC 控制核心实现。
 *
 * 本文件把整条控制链串起来：
 * `main.c` 给目标值 -> 本文件运行位置/速度/Iq PID ->
 * 根据 AS5600 角度计算电角度 -> 根据 Uq 和电角度计算三相 PWM ->
 * `hhPwm.c` 写入 TIM2/TIM3 输出。
 *
 * 学习主线：
 * 1. 先看 `hhFocInit()` 明白所有外设和全局参数如何初始化；
 * 2. 再看 `_electricalAngle_M0/M1()` 明白机械角度如何转电角度；
 * 3. 再看 `setPhaseVoltage()` 明白 Uq 如何变成 SPWM/SVPWM；
 * 4. 最后看 `hhCloseLoopPos_WithVelIqM0()` 这类串级闭环函数。
 */

struct Motor Motor0 = { 0 };
struct Motor Motor1 = { 1 };
/* 母线供电电压，单位 V；用于限制 Uq 和把相电压换算成占空比。 */
float voltage_power_supply;
/* 电机极对数：机械角度 * PP = 电角度。 */
int PP;
/* 编码器/电机方向修正系数，写入 AS5600 状态对象。 */
int DIR;

//限制只能输出low~high之间的数值
/**
 * @brief 把数值限制在指定上下限内。
 *
 * @param amt 原始输入值。
 * @param low 下限。
 * @param high 上限。
 * @return 限幅后的值。
 *
 * 用途：
 * - 限制 Uq 不超过母线电压一半；
 * - 限制三相电压在 0~母线电压之间；
 * - 限制 SPWM 占空比在 0~1 之间。
 */
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
/**
 * @brief 把角度归一化到 0~2π。
 *
 * @param angle 输入角度，单位 rad，可以为负或超过 2π。
 * @return 归一化后的角度，单位 rad，范围 [0, 2π)。
 *
 * @note FOC 中的三角函数只关心电角度在一圈内的位置，归一化可避免角度无限增长。
 */
float _normalizeAngle(float angle) {
	float a = fmod(angle, 2 * M_PI);  //取余保证角度是-360~360度
	if (a >= 0) {
		return a;
	} else {
		return a + 2 * M_PI;
	}
}

//由计算出三相电压设定PWM
/**
 * @brief 把已经计算好的三相电压写成 SPWM 占空比。
 *
 * 所在流程：
 * `setPhaseVoltage()` 的 SPWM 分支先计算 `Motor->Ua/Ub/Uc`，
 * 再调用本函数完成“电压 -> 占空比 -> 定时器比较值”的最后一步。
 *
 * @param Motor 指向 M0 或 M1 的电机输出状态。
 *
 * 流程：
 * 1. 限制三相目标电压不低于 0、不高于母线电压；
 * 2. 用相电压除以母线电压得到 0~1 占空比；
 * 3. 根据电机编号调用 `hhPwm.c` 中对应的 PWM 写入函数。
 */
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

/**
 * @brief FOC 电压调制核心：把 q/d 轴电压转换为三相 PWM 输出。
 *
 * 所在流程：
 * - 开环函数直接给 Uq 和电角度；
 * - 位置/速度/Iq 闭环通过 PID 算出 Uq；
 * - 本函数将 Uq 投影到 α/β，再根据当前 PWM 模式走 SPWM 或 SVPWM。
 *
 * @param Motor 指向 M0 或 M1 的电机输出状态。
 * @param Uq q 轴电压，单位 V；在表贴永磁电机中主要对应转矩输出。
 * @param Ud d 轴电压，单位 V；当前实现没有把 Ud 加入逆 Park 公式。
 * @param angle_el 当前电角度，单位 rad。
 *
 * SPWM 流程：
 * 1. 逆 Park：Uq + 电角度 -> Ualpha/Ubeta；
 * 2. 逆 Clarke：Ualpha/Ubeta -> Ua/Ub/Uc；
 * 3. 三相电压整体加 `Vbus/2`，让单片机 PWM 能用 0~Vbus 的正电压表达交流相电压；
 * 4. `SetPwm()` 把相电压换成占空比写入定时器。
 *
 * SVPWM 流程：
 * 1. 根据 Ualpha/Ubeta 判断空间矢量所在扇区；
 * 2. 计算相邻有效矢量作用时间 Tx/Ty；
 * 3. 分配零矢量时间，得到三相比较值 Tcmp1/2/3；
 * 4. 直接把比较值写入 TIM2/TIM3 的 CCR。
 *
 * @note 本函数是理解工程的核心，基本上所有闭环控制最后都会走到这里。
 */
void setPhaseVoltage(struct Motor *Motor, float Uq, float Ud, float angle_el) {
	/* 先把 Uq 限制在母线电压的一半以内，避免电压矢量过大导致占空比越界。 */
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
		  // PWM为高电平时：上桥臂MOS管导通，下桥臂MOS管关断，将电源电压施加到相应的绕组上。
		  // PWM为低电平时：上桥臂MOS管关断，在上桥臂关断后，稍等一段时间(死区时间，防止上下桥臂有短暂的导通造成短路)再让下桥臂导通。
		  //也就是说某一路PWM(都是正电压没有负电压)为高电平时，上桥臂导通，对应Ua线圈电流流入，这时Ua为正电压;PWM(都是正电压没有负电压)为低电平时,上桥臂关断,下桥臂导通。此时有可能在Ua线圈电流流出，这时Ua为负电压。
		SetPwm(Motor);
	}else{
		//SVPWM
		//1.扇区判断
		  /*
		   * 扇区判断的本质：
		   * 空间矢量平面被 6 个 60° 扇区分割。这里通过三个边界函数的正负
		   * 组合得到 N，再用 N 映射到实际扇区。后续 Tx/Ty 公式按 N 分支选择。
		   */
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
		  /*
		   * Tx/Ty 表示目标电压矢量在当前扇区相邻两个有效矢量上的作用时间。
		   * Ts 使用 `htim2.Init.Period * 2`，是因为中心对齐模式下上数和下数
		   * 共同构成一个完整 PWM 周期。
		   */
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
		  /*
		   * Ta/Tb/Tc 是把 Tx/Ty 和零矢量时间组合后的三个时间点。
		   * Tcmp1/2/3 根据扇区重新排列，对应三相最终写入的 CCR。
		   */
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

/**
 * @brief 根据 M0 机械单圈角度计算当前电角度。
 * @return M0 电角度，单位 rad，范围 0~2π。
 *
 * 公式：
 * `电角度 = 极对数 * 机械角度 - 零电角度偏置`。
 *
 * @note 使用单圈角度 `getAngle_Without_trackM0()` 即可，因为电角度本身按 2π 周期重复。
 */
float _electricalAngle_M0() {
  return _normalizeAngle((float) PP *getAngle_Without_trackM0() - zero_electric_angle_M0);
}

/**
 * @brief 根据 M1 机械单圈角度计算当前电角度。
 * @return M1 电角度，单位 rad，范围 0~2π。
 */
float _electricalAngle_M1() {
  return _normalizeAngle((float) PP *getAngle_Without_trackM1() - zero_electric_angle_M1);
}

/**
 * @brief 对齐 M0 电角度零点。
 *
 * 所在流程：
 * `hhFocInit()` 调用本函数。它给定一个固定电压矢量，让转子被吸到一个
 * 已知电角度方向，然后读取传感器角度并记录为零点偏置。
 *
 * 流程：
 * 1. 在 `_3PI_2` 方向施加 Uq=3V；
 * 2. 等待转子稳定；
 * 3. 读取 AS5600 并计算当前电角度；
 * 4. 保存为 `zero_electric_angle_M0`；
 * 5. 输出 0V，释放电机。
 */
void AlignElectricalAngleM0(){
	  setPhaseVoltage(&Motor0, 3, 0, _3PI_2);
	  HAL_Delay(1000);
	  Sensor_update();
	  zero_electric_angle_M0 = _electricalAngle_M0();
	  setPhaseVoltage(&Motor0, 0, 0, _3PI_2);
	  HAL_Delay(1000);
	  Serial_Printf("%s\r\n", "0电角度：",getAngleM0());
}

/**
 * @brief 对齐 M1 电角度零点。
 *
 * 流程与 `AlignElectricalAngleM0()` 相同，只是作用对象换成 M1。
 */
void AlignElectricalAngleM1(){
	  setPhaseVoltage(&Motor1, 3, 0, _3PI_2);
	  HAL_Delay(1000);
	  Sensor_update();
	  zero_electric_angle_M1 = _electricalAngle_M1();
	  setPhaseVoltage(&Motor1, 0, 0, _3PI_2);
	  HAL_Delay(1000);
	  Serial_Printf("%s\r\n", "1电角度：",getAngleM1());
}

/**
 * @brief M0 电角度对齐测试函数：按当前电角度输出固定 Uq。
 * @note 可用于观察电角度计算和相序是否正确，正常主流程未调用。
 */
void AlignElectricalTest0() {  //闭环位置
	setPhaseVoltage(&Motor0, 3, 0, _electricalAngle_M0());
}

/**
 * @brief M1 电角度对齐测试函数：按当前电角度输出固定 Uq。
 * @note 可用于观察电角度计算和相序是否正确，正常主流程未调用。
 */
void AlignElectricalTest1() {  //闭环位置
	setPhaseVoltage(&Motor1, 3, 0, _electricalAngle_M1());
}

/**
 * @brief 运行时重新配置定时器计数模式。
 *
 * @param hhTim 目标定时器句柄。
 * @param counterMode 目标计数模式，例如向上计数或中心对齐。
 * @return HAL_OK 表示重新初始化成功，其它值表示 HAL 初始化失败。
 *
 * 用途：
 * - SPWM 更适合普通向上计数；
 * - SVPWM 当前使用中心对齐模式。
 *
 * @note 当前函数参数名是 `hhTim`，但内部调用 `HAL_TIM_Base_Init(&htim2)`；
 *       学习时应按“当前代码实际行为”理解。
 */
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

/**
 * @brief 运行时重新配置某个 PWM 通道的输出比较模式。
 *
 * @param hhTim 目标定时器句柄。
 * @param channel 目标通道，如 `TIM_CHANNEL_1`。
 * @param ocMode 输出比较模式，如 `TIM_OCMODE_PWM1` 或 `TIM_OCMODE_PWM2`。
 * @return 当前代码无论 HAL 配置是否失败都会返回 `status` 初始值。
 *
 * 用途：
 * `hhChangeSpwmModel()` / `hhChangeSVpwmModel()` 用它在 SPWM 与 SVPWM
 * 两种 PWM 模式之间切换通道配置。
 */
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

/**
 * @brief 切换两台电机到 SPWM 模式。
 *
 * SPWM 模式下，`setPhaseVoltage()` 会先计算三相电压和归一化占空比，
 * 再通过 `M0SetPwmX()` / `M1SetPwmX()` 写入 PWM。
 *
 * 配置动作：
 * - Motor0/Motor1 的 `pwmMode` 设为 `PWM_MODE_SPWM`；
 * - 将相关定时器配置成向上计数；
 * - 将通道输出比较模式切到 PWM1。
 */
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

/**
 * @brief 切换两台电机到 SVPWM 模式。
 *
 * SVPWM 模式下，`setPhaseVoltage()` 不走三相占空比换算，
 * 而是直接计算每相 CCR 比较值并写入定时器。
 *
 * 配置动作：
 * - Motor0/Motor1 的 `pwmMode` 设为 `PWM_MODE_SVPWM`；
 * - 将相关定时器配置成中心对齐；
 * - 将通道输出比较模式切到 PWM2。
 *
 * @note 当前 M1 分支中有对 `htim4/TIM_CHANNEL_1` 的通道配置调用；
 *       `htim4` 在本工程主要作为 1 ms 基准定时器，阅读时要留意这一点。
 */
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

/**
 * @brief 初始化整套 FOC 控制链路。
 *
 * 所在流程：
 * `main.c` 在 HAL、GPIO、DMA、USART、TIM、ADC 初始化后调用本函数。
 * 本函数完成后，主循环就可以在 1 ms 节拍中调用传感器刷新和闭环控制。
 *
 * @param power_supply 母线供电电压，单位 V，用于电压限幅和 PWM 占空比换算。
 * @param _PP 电机极对数，机械角度转换成电角度时使用。
 * @param _DIR 方向修正系数，写入 AS5600 状态对象。
 * @param pwmMode 初始 PWM 调制方式，SPWM 或 SVPWM。
 *
 * 初始化顺序：
 * 1. 根据 `pwmMode` 切换定时器/PWM 模式；
 * 2. 保存极对数、方向和母线电压；
 * 3. 启动 PWM 输出；
 * 4. 启动串口接收，便于主循环读取上位机目标；
 * 5. 初始化 AS5600 软件 I2C；
 * 6. 启动 TIM4 基准中断，产生 1 ms 控制节拍；
 * 7. 设置 AS5600 方向和速度滤波参数；
 * 8. 分别对 M0/M1 做电角度零点对齐；
 * 9. 初始化 ADC 电流采样与 Iq 计算；
 * 10. 串口打印初始化完成信息。
 */
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

/**
 * @brief M0 直接输出指定 q/d 轴电压。
 * @param Uq q 轴电压，单位 V。
 * @param Ud d 轴电压，单位 V。
 * @param angle_el 电角度，单位 rad。
 */
void SetsetPhaseVoltageM0(float Uq, float Ud, float angle_el) {
	setPhaseVoltage(&Motor0, Uq, Ud, angle_el);
}

/**
 * @brief M1 直接输出指定 q/d 轴电压。
 * @param Uq q 轴电压，单位 V。
 * @param Ud d 轴电压，单位 V。
 * @param angle_el 电角度，单位 rad。
 */
void SetsetPhaseVoltageM1(float Uq, float Ud, float angle_el) {
	setPhaseVoltage(&Motor1, Uq, Ud, angle_el);
}

float OpenLoopShaftAngle=0;

/**
 * @brief M0 开环速度控制：不看传感器反馈，只按目标速度推进电角度。
 *
 * @param target_velocity 目标机械角速度，单位 rad/s。
 *
 * 流程：
 * 1. 假设控制周期为 1 ms；
 * 2. 机械角度累加 `target_velocity * Ts`；
 * 3. 用极对数把机械角度换算成电角度；
 * 4. 固定输出 `Vbus/3` 的 Uq 让电机转动。
 *
 * @note 开环用于调试相序或无传感器前的简单拖动，不会纠正丢步或负载扰动。
 */
void hhOpenLoopVelM0(float target_velocity){
  //计算当前每个Loop的运行时间间隔
  float Ts = 1 * 1e-3f;
  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  OpenLoopShaftAngle = _normalizeAngle(OpenLoopShaftAngle + target_velocity*Ts);

  float Uq = voltage_power_supply/3;
  SetsetPhaseVoltageM0(Uq,  0, PP * OpenLoopShaftAngle);
}

float OpenLoopShaftAngle1=0;

/**
 * @brief M1 开环速度控制：不看传感器反馈，只按目标速度推进电角度。
 * @param target_velocity 目标机械角速度，单位 rad/s。
 */
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
/**
 * @brief M0 简单比例位置闭环，位置误差直接换算成 Uq
 *
 * @param Target 目标机械角度，单位 rad
 * @note 这是早期/简化版本，未使用 `hhPID` 对象，输出限幅固定为 -6~6 V
 */
void hhCloseLoopPos(float Target) {  
		setPhaseVoltage(&Motor0, _constrain(0.133 * (Target-getAngleM0())*180/M_PI,-6,6), 0, _electricalAngle_M0());
}

/**
 * @brief M1 简单比例位置闭环，位置误差直接换算成 Uq
 * @param Target 目标机械角度，单位 rad
 */
void hhCloseLoopPos1(float Target) {  //闭环位置
		setPhaseVoltage(&Motor1, _constrain(0.133 * (Target-getAngleM1())*180/M_PI,-6,6), 0, _electricalAngle_M1());
}




////////////////////////////////////////////////////////
////////////////闭环速度PID//////////////////////////////////
struct hhPID hhClosLoopVelPID_Obj_M0 ;

/**
 * @brief 设置 M0 速度环 PID 参数。
 *
 * @param mP 比例增益。
 * @param mI 积分增益。
 * @param mD 微分增益。
 * @param mramp PID 输出变化率限制，单位取决于输出量/秒。
 * @param mlimit PID 输出幅值限制。
 *
 * @note M0 速度环的输出在不同调用路径中含义不同：
 *       直接速度闭环时输出 Uq；速度+Iq 串级时输出目标 Iq。
 */
void hhSetCloseLoopVel_PIDM0(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopVelPID_Obj_M0.P = mP;
	hhClosLoopVelPID_Obj_M0.I = mI;
	hhClosLoopVelPID_Obj_M0.D = mD;
	hhClosLoopVelPID_Obj_M0.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopVelPID_Obj_M0.limit = mlimit;  // PID控制器输出限幅
}

/**
 * @brief M0 速度闭环，速度误差经 PID 后直接作为 Uq 输出。
 *
 * @param Target 目标机械角速度，单位 rad/s。
 *
 * 流程：
 * 1. `GetFilterV_M0()` 获取滤波后的实际速度；
 * 2. 目标速度减实际速度得到速度误差；
 * 3. 当前代码把 rad/s 误差乘 `180/π` 转成“度每秒”尺度输入 PID；
 * 4. PID 输出直接作为 q 轴电压 Uq；
 * 5. 用当前 M0 电角度调用 `setPhaseVoltage()` 输出三相 PWM。
 */
void hhCloseLoopVelM0(float Target) {  //闭环速度
  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopVelPID_Obj_M0,(Target - GetFilterV_M0()) * 180 / _PI), 0, _electricalAngle_M0());
}

struct hhPID hhClosLoopVelPID_Obj_M1 ;

/**
 * @brief 设置 M1 速度环 PID 参数。
 */
void hhSetCloseLoopVel_PIDM1(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopVelPID_Obj_M1.P = mP;
	hhClosLoopVelPID_Obj_M1.I = mI;
	hhClosLoopVelPID_Obj_M1.D = mD;
	hhClosLoopVelPID_Obj_M1.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopVelPID_Obj_M1.limit = mlimit;  // PID控制器输出限幅
}

/**
 * @brief M1 速度闭环，速度误差经 PID 后直接作为 Uq 输出。
 * @param Target 目标机械角速度，单位 rad/s。
 */
void hhCloseLoopVelM1(float Target) {  //闭环速度
  setPhaseVoltage(&Motor1, PidRun(&hhClosLoopVelPID_Obj_M1,(Target - GetFilterV_M1()) * 180 / _PI), 0, _electricalAngle_M1());
}






////////////////////////////////////////////////////////
////////////////闭环位置PID//////////////////////////////////
struct hhPID hhClosLoopPosPID_Obj_M0 ;

/**
 * @brief 设置 M0 位置环 PID 参数。
 *
 * @param mP 比例增益。
 * @param mI 积分增益。
 * @param mD 微分增益。
 * @param mramp PID 输出变化率限制。
 * @param mlimit PID 输出幅值限制。
 *
 * @note M0 位置环输出在不同串级结构中含义不同：
 *       直接位置闭环时输出 Uq；位置+速度+Iq 时输出目标速度
 */
void hhSetCloseLoopPos_PIDM0(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopPosPID_Obj_M0.P = mP;
	hhClosLoopPosPID_Obj_M0.I = mI;
	hhClosLoopPosPID_Obj_M0.D = mD;
	hhClosLoopPosPID_Obj_M0.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopPosPID_Obj_M0.limit = mlimit;  // PID控制器输出限幅
}

/**
 * @brief M0 位置闭环，位置误差经 PID 后直接作为 Uq 输出。
 *
 * @param Target 目标机械角度，单位 rad
 * @note 函数旁原注释写“闭环速度”，但从误差 `(Target-getAngleM0())` 看实际是位置闭环
 */
void hhCloseLoopPosM0(float Target) {  
  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/_PI), 0, _electricalAngle_M0());
}

struct hhPID hhClosLoopPosPID_Obj_M1 ;

/**
 * @brief 设置 M1 位置环 PID 参数。
 */
void hhSetCloseLoopPos_PIDM1(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopPosPID_Obj_M1.P = mP;
	hhClosLoopPosPID_Obj_M1.I = mI;
	hhClosLoopPosPID_Obj_M1.D = mD;
	hhClosLoopPosPID_Obj_M1.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopPosPID_Obj_M1.limit = mlimit;  // PID控制器输出限幅
}

/**
 * @brief M1 位置闭环，位置误差经 PID 后直接作为 Uq 输出
 * @param Target 目标机械角度，单位 rad
 */
void hhCloseLoopPosM1(float Target) {  
  setPhaseVoltage(&Motor1, PidRun(&hhClosLoopPosPID_Obj_M1,(Target-getAngleM1())*180/M_PI), 0, _electricalAngle_M1());
}




////////////////////////////////////////////////////////
////////////////刷新传感器函数//////////////////////////////////
/**
 * @brief 在 1 ms 控制节拍中刷新所有反馈量
 *
 * 所在流程：
 * `main.c` 的 while 循环检测到 `Flag_1ms` 后调用本函数。
 *
 * 刷新顺序：
 * 1. `Sensor_update()` 读取两颗 AS5600，更新机械角度和速度；
 * 2. 用 M1 当前电角度刷新 M1 的 Iq；
 * 3. 用 M0 当前电角度刷新 M0 的 Iq。
 *
 * @note `IqSensorUpdateM1()` 会先把 DMA ADC 缓冲复制到 `AD_Value` 快照，
 *       因此 M1/M0 的 Iq 计算使用同一份 ADC 快照。
 */
void UpdateAllSensor(){
	Sensor_update();//刷新AS5600
	IqSensorUpdateM1(_electricalAngle_M1());//电机1 Iq电流获取
	IqSensorUpdateM0(_electricalAngle_M0());//电机0 Iq电流获取
}




////////////////////////////////////////////////////////
////////////////闭环Iq PID//////////////////////////////////
struct hhPID hhClosLoopIqPID_Obj_M0 ;

/**
 * @brief 设置 M0 Iq 电流环 PID 参数。
 *
 * @param mP 比例增益。
 * @param mI 积分增益。
 * @param mD 微分增益。
 * @param mramp PID 输出变化率限制。
 * @param mlimit PID 输出幅值限制，通常应限制在安全电压范围内。
 */
void hhSetCloseLoopIq_PIDM0(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopIqPID_Obj_M0.P = mP;
	hhClosLoopIqPID_Obj_M0.I = mI;
	hhClosLoopIqPID_Obj_M0.D = mD;
	hhClosLoopIqPID_Obj_M0.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopIqPID_Obj_M0.limit = mlimit;  // PID控制器输出限幅
}

/**
 * @brief M0 Iq 电流闭环，电流误差经 PID 后输出 Uq。
 *
 * @param Target 目标 q 轴电流，单位 A。
 *
 * 流程：
 * 1. `GetFilterIqM0()` 获取滤波后实际 Iq；
 * 2. 目标 Iq 减实际 Iq 得到电流误差；
 * 3. Iq PID 输出 q 轴电压 Uq；
 * 4. 按当前电角度输出三相 PWM。
 */
void hhCloseLoopIqM0(float Target) {  
  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopIqPID_Obj_M0,Target - GetFilterIqM0() ), 0, _electricalAngle_M0());
}

struct hhPID hhClosLoopIqPID_Obj_M1 ;

/**
 * @brief 设置 M1 Iq 电流环 PID 参数
 */
void hhSetCloseLoopIq_PIDM1(float mP, float mI, float mD, float mramp, float mlimit) {
	hhClosLoopIqPID_Obj_M1.P = mP;
	hhClosLoopIqPID_Obj_M1.I = mI;
	hhClosLoopIqPID_Obj_M1.D = mD;
	hhClosLoopIqPID_Obj_M1.output_ramp = mramp;               // PID控制器加速度限幅
	hhClosLoopIqPID_Obj_M1.limit = mlimit;  // PID控制器输出限幅
}

/**
 * @brief M1 Iq 电流闭环，电流误差经 PID 后输出 Uq。
 * @param Target 目标 q 轴电流，单位 A。
 */
void hhCloseLoopIqM1(float Target) {  //闭环速度
  setPhaseVoltage(&Motor1, PidRun(&hhClosLoopIqPID_Obj_M1,Target - GetFilterIqM1() ), 0, _electricalAngle_M1());
}



////////////////////////////////////////////////////////
////////////////闭环速度 闭环Iq PID//////////////////////////////////
/**
 * @brief M0 速度环 + Iq 电流环串级闭环
 *
 * @param Target 目标机械角速度，单位 rad/s。
 *
 * 串级关系：
 * - 外环：速度误差 -> 速度 PID -> 目标 Iq；
 * - 内环：目标 Iq - 实际 Iq -> Iq PID -> Uq -> PWM。
 *
 * 这样比“速度 PID 直接输出 Uq”更接近真实电机控制，因为内层电流环会约束转矩电流 
 */
void hhCloseLoopVel_WithIqM0(float Target) {  //带电流环的闭环速度

  // setPhaseVoltage(&Motor0, PidRun(&hhClosLoopVelPID_Obj_M0,(Target - GetFilterV_M0()) * 180 / M_PI), 0, _electricalAngle_M0())

  // 电流环所需要的电流目标值由速度环 PID 输出
  hhCloseLoopIqM0(PidRun(&hhClosLoopVelPID_Obj_M0,(Target - GetFilterV_M0()) * 180 / _PI)) ;//改进后
}

/**
 * @brief M1 速度环 + Iq 电流环串级闭环。
 * @param Target 目标机械角速度，单位 rad/s。
 */
void hhCloseLoopVel_WithIqM1(float Target) {  //带电流环的闭环速度
  // setPhaseVoltage(&Motor0, PidRun(&hhClosLoopVelPID_Obj_M0,(Target - GetFilterV_M0()) * 180 / M_PI), 0, _electricalAngle_M0());//改进前
  hhCloseLoopIqM1(PidRun(&hhClosLoopVelPID_Obj_M1,(Target - GetFilterV_M1()) * 180 / _PI)) ;//改进后
}





////////////////////////////////////////////////////////
////////////////闭环位置 闭环Iq PID//////////////////////////////////
/**
 * @brief M0 位置环 + Iq 电流环串级闭环
 *
 * @param Target 目标机械角度，单位 rad
 *
 * 串级关系：
 * - 外环：位置误差 -> 位置 PID -> 目标 Iq；
 * - 内环：目标 Iq -> Iq PID -> Uq -> PWM
 *
 * @note 该结构省略速度环，位置环输出直接当作电流目标
 */
void hhCloseLoopPos_WithIqM0(float Target) {  //带电流环的闭环位置
  //  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/M_PI), 0, _electricalAngle_M0());//改进前


	hhCloseLoopIqM0(PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/_PI));  //改进后
}

/**
 * @brief M1 位置环 + Iq 电流环串级闭环。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPos_WithIqM1(float Target) {  //带电流环的闭环位置
  //  setPhaseVoltage(&Motor0, PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/M_PI), 0, _electricalAngle_M0());//改进前
	hhCloseLoopIqM1(PidRun(&hhClosLoopPosPID_Obj_M1,(Target-getAngleM1())*180/_PI));  //改进后
}



////////////////////////////////////////////////////////
////////////////闭环位置 闭环速度 闭环Iq PID//////////////////////////////////
/**
 * @brief M0 位置环 + 速度环 + Iq 电流环三级串级闭环。
 *
 * @param Target 目标机械角度，单位 rad。
 *
 * 这是 `main.c` 当前实际调用的主控制函数。
 *
 * 三级关系：
 * 1. 位置误差 `Target - getAngleM0()` 输入位置 PID，输出目标速度；
 * 2. 目标速度减实际滤波速度 `GetFilterV_M0()`，输入速度 PID，输出目标 Iq；
 * 3. 目标 Iq 输入 `hhCloseLoopIqM0()`，由电流环输出 Uq；
 * 4. `setPhaseVoltage()` 根据 Uq 和电角度生成 PWM。
 *
 * @note 位置误差乘 `180/π`，实际是把 rad 量纲转换为 degree 尺度后再进入 PID。
 */
void hhCloseLoopPos_WithVelIqM0(float Target) {
	hhCloseLoopIqM0(PidRun(&hhClosLoopVelPID_Obj_M0,PidRun(&hhClosLoopPosPID_Obj_M0,(Target-getAngleM0())*180/_PI)-GetFilterV_M0()));
}

/**
 * @brief M1 位置环 + 速度环 + Iq 电流环三级串级闭环。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPos_WithVelIqM1(float Target) {
	hhCloseLoopIqM1(PidRun(&hhClosLoopVelPID_Obj_M1,PidRun(&hhClosLoopPosPID_Obj_M1,(Target-getAngleM1())*180/_PI)-GetFilterV_M1()));
}






