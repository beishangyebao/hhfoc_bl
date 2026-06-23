/*
 * hhPID.h
 *
 *  Created on: Mar 15, 2025
 *      Author: KingPC
 */

#ifndef HHPID_HHPID_H_
#define HHPID_HHPID_H_
struct hhPID{
	  float P;            //比例(P环增益)
	  float I;            //积分(I环增益)
	  float D;            //微分(D环增益)
	  float output_ramp;  // PID控制器加速度限幅
	  float limit;        // PID控制器输出限幅
	  float error_prev;              //最后的跟踪误差值
	  float output_prev;             //最后一个 pid 输出值
	  float integral_prev;           //最后一个积分分量值
};
float PidRun(struct hhPID *hhPIDP,float error);


#endif /* HHPID_HHPID_H_ */
