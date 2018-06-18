/*
 * filter.h
 *
 *  Created on: Jun 7, 2018
 *      Author: seb
 */

#ifndef FILTER_H_
#define FILTER_H_

#include "sapi.h"       // <= sAPI header


typedef uint16_t filter_order_t;
typedef float filter_type_t;

typedef struct filter_s{
	float *vecB;
	float *vecA;
	filter_type_t *vecX;
	filter_type_t *vecY;
	filter_order_t order;
}filter_t;

uint8_t Filter_Init(filter_t *filter, float *vecB, float *vecA, filter_type_t *vecX, filter_type_t *vecY, filter_order_t order);
filter_type_t Filter_Process(filter_t *filter, filter_type_t sample);
uint8_t Filter_Reset(filter_t *filter);

#endif /* FILTER_H_ */
