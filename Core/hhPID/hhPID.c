/*
 * hhPID.c
 *
 *  Created on: Mar 15, 2025
 *      Author: KingPC
 */

#include"hhPID.h"

// 限幅
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
  /*
   *  Tustin I环
   *  用梯形面积代替长方形面积
   *  上底是上一拍误差 下底是当前拍误差 高度是控制周期
   *  最终当前节拍误差就是梯形面积
   */
  float integral = hhPIDP->integral_prev + hhPIDP->I * Ts * 0.5f * (error + hhPIDP->error_prev);
  //限幅
  integral = constrain(integral, -hhPIDP->limit, hhPIDP->limit);

  // D环（微分环节）
  /*
   * D项看误差变化率
   * 误差变化率 = (当前误差 - 上一次误差) / 控制周期
   */
  float derivative = hhPIDP->D * (error - hhPIDP->error_prev) / Ts;

  // 将P,I,D三环的计算值加起来
  float output = proportional + integral + derivative;
  // 限幅
  output = constrain(output, -hhPIDP->limit, hhPIDP->limit);

  // 对PID的变化速率进行限制
  if (hhPIDP->output_ramp > 0) {

    // 计算变化率
    float output_rate = (output - hhPIDP->output_prev) / Ts;

    // 如果变化率大于正向最大变化率
    if (output_rate > hhPIDP->output_ramp)

      // 变化率 = 上一拍变化率 + 最大变化率 * 控制周期
      output = hhPIDP->output_prev + hhPIDP->output_ramp * Ts;

    //反向同理
    else if (output_rate < -hhPIDP->output_ramp)
      output = hhPIDP->output_prev - hhPIDP->output_ramp * Ts;
  }

  // 保存值（为了下一次循环）
  hhPIDP->integral_prev = integral;
  hhPIDP->output_prev = output;
  hhPIDP->error_prev = error;
  return output;
}


