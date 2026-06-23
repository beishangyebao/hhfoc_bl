/*
 * hhPwm.h
 *
 *  Created on: Mar 11, 2025
 *      Author: KingPC
 */

#ifndef HHPWM_HHPWM_H_
#define HHPWM_HHPWM_H_

void InitPwm();
void M0SetPwmA(float Duty);
void M0SetPwmB(float Duty);
void M0SetPwmC(float Duty);
void M1SetPwmA(float Duty);
void M1SetPwmB(float Duty);
void M1SetPwmC(float Duty);

void M0SVPWMSetPwmA(uint32_t CCR);
void M0SVPWMSetPwmB(uint32_t CCR);
void M0SVPWMSetPwmC(uint32_t CCR);

void M1SVPWMSetPwmA(uint32_t CCR);
void M1SVPWMSetPwmB(uint32_t CCR);
void M1SVPWMSetPwmC(uint32_t CCR);
#endif /* HHPWM_HHPWM_H_ */
