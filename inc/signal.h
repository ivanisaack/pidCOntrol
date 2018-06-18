/*
 * signal.h
 *
 *  Created on: Jun 7, 2018
 *      Author: seb
 */

#ifndef SIGNAL_H_
#define SIGNAL_H_

#define sig_len		(200)

#include "sapi.h"

float Signal_Sample(uint32_t index);
void Signal_Init(void);

#endif /* SIGNAL_H_ */
