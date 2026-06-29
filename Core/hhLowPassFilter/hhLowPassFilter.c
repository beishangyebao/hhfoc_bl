
#include"hhLowPassFilter.h"

/*
 * 一阶低通滤波器
 * y[n] = alpha * y[n-1] + (1-alpha) * x[n]
 * x：当前输入的原始采样值
 * y：滤波后的采样值
 * alpha：滤波系数，取值范围为0~1，越接近1，滤波效果越明显，但响应速度越慢
 * Tf：滤波时间常数 决定滤波器的截止频率
 *     f = 1 / (2 * PI * Tf)
 * dt：采样周期
 */
float hhGetFilterValue(struct hhLowPassFilter *hhLowPassFilterP,float x) {
   // alpha = Tf / (Tf + dt)
  float alpha = hhLowPassFilterP->Tf / (hhLowPassFilterP->Tf + hhLowPassFilterP->dt);
  float y = alpha * hhLowPassFilterP->y_prev + (1.0f - alpha) * x;
  hhLowPassFilterP->y_prev = y;
  return y;
}


