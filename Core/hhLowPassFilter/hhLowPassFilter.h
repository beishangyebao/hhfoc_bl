/*
 * hhLowPassFilter.h
 *
 *  Created on: Mar 15, 2025
 *      Author: KingPC
 */

#ifndef HHLOWPASSFILTER_HHLOWPASSFILTER_H_
#define HHLOWPASSFILTER_HHLOWPASSFILTER_H_

struct hhLowPassFilter{
	float dt;
	float Tf;
	float y_prev ;
};
float hhGetFilterValue(struct hhLowPassFilter *hhLowPassFilterP,float x);

#endif /* HHLOWPASSFILTER_HHLOWPASSFILTER_H_ */
