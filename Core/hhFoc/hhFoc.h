/*
 * hhFoc.h
 *
 *  Created on: Mar 12, 2025
 *      Author: KingPC
 */

#ifndef HHFOC_HHFOC_H_
#define HHFOC_HHFOC_H_
typedef enum {
    PWM_MODE_SPWM = 0,
    PWM_MODE_SVPWM
} pwm_mode_t;

struct Motor{
	int Mot_num;
	float zero_electric_angle;
	float Ua;
	float Ub;
	float Uc;
	float Ubeta;
	float Ualpha;
	float dc_a;
	float dc_b;
	float dc_c;
	pwm_mode_t  pwmMode;
};

void hhFocInit(float power_supply, int _PP, int _DIR,pwm_mode_t pwmMode);
void SetsetPhaseVoltageM0( float Uq, float Ud, float angle_el);
void SetsetPhaseVoltageM1( float Uq, float Ud, float angle_el);
void hhOpenLoopVelM0(float target_velocity);
void hhOpenLoopVelM1(float target_velocity);
void hhCloseLoopPos(float Target);
void hhCloseLoopPos1(float Target);
void hhSetCloseLoopVel_PIDM0(float mP, float mI, float mD, float mramp, float mlimit);
void hhSetCloseLoopVel_PIDM1(float mP, float mI, float mD, float mramp, float mlimit);
void hhCloseLoopVelM0(float Target);
void hhCloseLoopVelM1(float Target);
void hhSetCloseLoopPos_PIDM0(float mP, float mI, float mD, float mramp, float mlimit);
void hhSetCloseLoopPos_PIDM1(float mP, float mI, float mD, float mramp, float mlimit);
void hhCloseLoopPosM0(float Target);
void hhCloseLoopPosM1(float Target);
void UpdateAllSensor();
void hhSetCloseLoopIq_PIDM0(float mP, float mI, float mD, float mramp, float mlimit);
void hhSetCloseLoopIq_PIDM1(float mP, float mI, float mD, float mramp, float mlimit);
void hhCloseLoopIqM0(float Target);
void hhCloseLoopIqM1(float Target);
void hhCloseLoopVel_WithIqM0(float Target);
void hhCloseLoopVel_WithIqM1(float Target);
void hhCloseLoopPos_WithIqM0(float Target) ;
void hhCloseLoopPos_WithIqM1(float Target) ;
void hhCloseLoopPos_WithVelIqM0(float Target);
void hhCloseLoopPos_WithVelIqM1(float Target);
#endif /* HHFOC_HHFOC_H_ */
