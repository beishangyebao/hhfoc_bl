/*
 * hhPID.c
 *
 *  Created on: Mar 15, 2025
 *      Author: KingPC
 */

#include"hhPID.h"
double constrain(double amt, double low, double high) {
  if (amt < low) {
    return low;
  } else if (amt > high) {
    return high;
  } else {
    return amt;
  }
}
float PidRun(struct hhPID *hhPIDP,float error) {
  // 两次循环中间的间隔时间
  float Ts = 1e-3f; //1ms
  // P环
  float proportional = hhPIDP->P * error;
  //  散点积分Tustin（I环）
  float integral = hhPIDP->integral_prev + hhPIDP->I * Ts * 0.5f * (error + hhPIDP->error_prev);
  integral = constrain(integral, -hhPIDP->limit, hhPIDP->limit);
  // D环（微分环节）
  float derivative = hhPIDP->D * (error - hhPIDP->error_prev) / Ts;

  // 将P,I,D三环的计算值加起来
  float output = proportional + integral + derivative;
  output = constrain(output, -hhPIDP->limit, hhPIDP->limit);

  if (hhPIDP->output_ramp > 0) {
    // 对PID的变化速率进行限制
    float output_rate = (output - hhPIDP->output_prev) / Ts;
    if (output_rate > hhPIDP->output_ramp)
      output = hhPIDP->output_prev + hhPIDP->output_ramp * Ts;
    else if (output_rate < -hhPIDP->output_ramp)
      output = hhPIDP->output_prev - hhPIDP->output_ramp * Ts;
  }
  // 保存值（为了下一次循环）
  hhPIDP->integral_prev = integral;
  hhPIDP->output_prev = output;
  hhPIDP->error_prev = error;
  return output;
}


