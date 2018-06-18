/*
 * filter.c
 *
 *  Created on: Jun 7, 2018
 *      Author: seb
 */

#include "filter.h"

uint8_t Filter_Init(filter_t *filter, float *vecB, float *vecA, filter_type_t *vecX, filter_type_t *vecY, filter_order_t order){
	filter->vecB = vecB;
	filter->vecA = vecA;
	filter->vecX = vecX;
	filter->vecY = vecY;
	filter->order = order;
	return 0;
}

uint8_t Filter_Reset(filter_t *filter){
	uint32_t i;
	uint32_t len;

	len = filter->order + 1;

	for(i = 0; i < len; i++){
		filter->vecX[i] = 0;
		filter->vecY[i] = 0;
	}
	return 0;
}

filter_type_t Filter_Process(filter_t *filter, filter_type_t x){
	float y;

	uint32_t i;
	uint32_t len;

	len = filter->order + 1;

	for(i = (len - 1); 1 <= i ; i--){
		filter->vecX[i] = filter->vecX[i - 1];
		filter->vecY[i] = filter->vecY[i - 1];
	}
	filter->vecX[0] = x;

	float Xi;
	float accB, Bi;
	accB = 0;

	for(i = 0; i < len; i++){
		Bi = filter->vecB[i];
		Xi = (float)filter->vecX[i];

		accB += Bi * Xi;
	}

	float Yi;
	float accA, Ao, Ai;
	Ao = filter->vecA[0];

	accA = 0.0;

	for(i = 1; i < len; i++){
		Ai = filter->vecA[i];
		Yi = filter->vecY[i];

		accA += Ai * Yi;
	}

	y = (accB - accA) / Ao;

	filter->vecY[0] = y;
	return y;
}
