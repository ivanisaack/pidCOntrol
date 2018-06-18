/*
 * pid.h
 *
 *  Created on: Jun 14, 2018
 *      Author: seb
 */

#ifndef PID_H_
#define PID_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct control_pid_s{
	float Kp;
	float Ki;
	float Kd;
	float T;
	float last_error;
	float acc;
}control_pid_t;

uint8_t PID_Init(control_pid_t *pid, float Kp, float Ki, float Kd, float T);
uint8_t PID_Reset(control_pid_t *pid);
float PID_Process(control_pid_t *pid, float error);

#endif /* PID_H_ */
