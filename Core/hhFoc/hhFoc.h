
#ifndef HHFOC_HHFOC_H_
#define HHFOC_HHFOC_H_

/**
 * @file hhFoc.h
 * @brief FOC 主控制接口：初始化、开环、位置环、速度环、Iq 电流环和 PWM 模式切换。
 *
 * 本模块是工程的控制核心，上游从 `main.c` 接收目标位置/速度/电流，
 * 下游通过 `hhAS5600` 读取角度、通过 `hhGetIq` 读取电流、通过
 * `hhPwm` 输出三相 PWM。
 */

/**
 * @brief PWM 调制模式选择。
 */
typedef enum {
    /** 正弦 PWM：由三相电压换算成 0~1 占空比。 */
    PWM_MODE_SPWM = 0,
    /** 空间矢量 PWM：直接计算三相 CCR 比较值。 */
    PWM_MODE_SVPWM
} pwm_mode_t;

/**
 * @brief 单台电机在 FOC 调制过程中的中间量。
 *
 * 该结构体保存的是“输出侧”状态：
 * - `Uq/Ud` 经过逆 Park 变换得到 `Ualpha/Ubeta`；
 * - 再经 SPWM 或 SVPWM 变成三相输出；
 * - SPWM 路径会保存 `Ua/Ub/Uc` 和 `dc_a/b/c`；
 * - SVPWM 路径主要使用 `Ualpha/Ubeta` 计算 CCR。
 */
struct Motor{
	/** 电机编号：0 表示 M0，非 0 表示 M1。 */
	int Mot_num;
	/** 电角度零点，当前实际使用全局 `zero_electric_angle_M0/M1`。 */
	float zero_electric_angle;
	/** A/B/C 三相目标电压，单位 V，主要用于 SPWM 分支。 */
	float Ua;
	float Ub;
	float Uc;
	/** α/β 静止坐标系电压，单位 V，由逆 Park 变换得到。 */
	float Ubeta;
	float Ualpha;
	/** A/B/C 三相归一化占空比，范围 0~1，主要用于 SPWM 分支。 */
	float dc_a;
	float dc_b;
	float dc_c;
	/** 当前使用 SPWM 还是 SVPWM。 */
	pwm_mode_t  pwmMode;
};

/**
 * @brief 初始化 FOC 控制链路和外设封装。
 * @param power_supply 母线供电电压，单位 V。
 * @param _PP 电机极对数。
 * @param _DIR 编码器/电机方向修正系数，通常为 1 或 -1。
 * @param pwmMode 初始 PWM 调制模式。
 */
void hhFocInit(float power_supply, int _PP, int _DIR,pwm_mode_t pwmMode);

/**
 * @brief 给 M0 直接设置 q/d 轴电压并输出三相 PWM。
 * @param Uq q 轴电压，单位 V，主要产生转矩。
 * @param Ud d 轴电压，单位 V，当前实现中参数未参与逆 Park 计算。
 * @param angle_el 电角度，单位 rad。
 */
void SetsetPhaseVoltageM0( float Uq, float Ud, float angle_el);

/**
 * @brief 给 M1 直接设置 q/d 轴电压并输出三相 PWM。
 * @param Uq q 轴电压，单位 V，主要产生转矩。
 * @param Ud d 轴电压，单位 V，当前实现中参数未参与逆 Park 计算。
 * @param angle_el 电角度，单位 rad。
 */
void SetsetPhaseVoltageM1( float Uq, float Ud, float angle_el);

/**
 * @brief M0 开环速度控制。
 * @param target_velocity 目标机械角速度，单位 rad/s。
 */
void hhOpenLoopVelM0(float target_velocity);

/**
 * @brief M1 开环速度控制。
 * @param target_velocity 目标机械角速度，单位 rad/s。
 */
void hhOpenLoopVelM1(float target_velocity);

/**
 * @brief M0 简单比例闭环位置控制。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPos(float Target);

/**
 * @brief M1 简单比例闭环位置控制。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPos1(float Target);

/**
 * @brief 设置 M0 速度环 PID 参数。
 */
void hhSetCloseLoopVel_PIDM0(float mP, float mI, float mD, float mramp, float mlimit);

/**
 * @brief 设置 M1 速度环 PID 参数。
 */
void hhSetCloseLoopVel_PIDM1(float mP, float mI, float mD, float mramp, float mlimit);

/**
 * @brief M0 速度闭环，速度环直接输出 Uq。
 * @param Target 目标机械角速度，单位 rad/s。
 */
void hhCloseLoopVelM0(float Target);

/**
 * @brief M1 速度闭环，速度环直接输出 Uq。
 * @param Target 目标机械角速度，单位 rad/s。
 */
void hhCloseLoopVelM1(float Target);

/**
 * @brief 设置 M0 位置环 PID 参数。
 */
void hhSetCloseLoopPos_PIDM0(float mP, float mI, float mD, float mramp, float mlimit);

/**
 * @brief 设置 M1 位置环 PID 参数。
 */
void hhSetCloseLoopPos_PIDM1(float mP, float mI, float mD, float mramp, float mlimit);

/**
 * @brief M0 位置闭环，位置环直接输出 Uq。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPosM0(float Target);

/**
 * @brief M1 位置闭环，位置环直接输出 Uq。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPosM1(float Target);

/**
 * @brief 刷新角度、速度和 Iq 电流采样。
 * @note 当前由 `main.c` 在 1 ms 控制节拍中调用。
 */
void UpdateAllSensor();

/**
 * @brief 设置 M0 Iq 电流环 PID 参数。
 */
void hhSetCloseLoopIq_PIDM0(float mP, float mI, float mD, float mramp, float mlimit);

/**
 * @brief 设置 M1 Iq 电流环 PID 参数。
 */
void hhSetCloseLoopIq_PIDM1(float mP, float mI, float mD, float mramp, float mlimit);

/**
 * @brief M0 Iq 电流闭环，电流环输出 Uq。
 * @param Target 目标 q 轴电流，单位 A。
 */
void hhCloseLoopIqM0(float Target);

/**
 * @brief M1 Iq 电流闭环，电流环输出 Uq。
 * @param Target 目标 q 轴电流，单位 A。
 */
void hhCloseLoopIqM1(float Target);

/**
 * @brief M0 速度环 + Iq 电流环串级闭环。
 * @param Target 目标机械角速度，单位 rad/s。
 */
void hhCloseLoopVel_WithIqM0(float Target);

/**
 * @brief M1 速度环 + Iq 电流环串级闭环。
 * @param Target 目标机械角速度，单位 rad/s。
 */
void hhCloseLoopVel_WithIqM1(float Target);

/**
 * @brief M0 位置环 + Iq 电流环串级闭环。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPos_WithIqM0(float Target) ;

/**
 * @brief M1 位置环 + Iq 电流环串级闭环。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPos_WithIqM1(float Target) ;

/**
 * @brief M0 位置环 + 速度环 + Iq 电流环三级串级闭环。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPos_WithVelIqM0(float Target);

/**
 * @brief M1 位置环 + 速度环 + Iq 电流环三级串级闭环。
 * @param Target 目标机械角度，单位 rad。
 */
void hhCloseLoopPos_WithVelIqM1(float Target);
#endif /* HHFOC_HHFOC_H_ */
