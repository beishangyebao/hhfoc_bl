/*
 * hhLowPassFilter.c
 *
 *  Created on: Mar 15, 2025
 *      Author: KingPC
 */

#include"hhLowPassFilter.h"
float hhGetFilterValue(struct hhLowPassFilter *hhLowPassFilterP,float x) {
  float alpha = hhLowPassFilterP->Tf / (hhLowPassFilterP->Tf + hhLowPassFilterP->dt);
  float y = alpha * hhLowPassFilterP->y_prev + (1.0f - alpha) * x;
  hhLowPassFilterP->y_prev = y;
  return y;
}


