/*
 * pid.c
 *
 *  Created on: Jun 14, 2018
 *      Author: seb
 */

#include "pid.h"

uint8_t PID_Init(control_pid_t *pid, float Kp, float Ki, float Kd, float T){
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->T = T;
	PID_Reset(pid);
	return 0;
}

uint8_t PID_Reset(control_pid_t *pid){
	pid->last_error = 0.0;
	pid->acc = 0.0;
	return 0;
}

float PID_Process(control_pid_t *pid, float error){
	float P;
	P = error * pid->Kp;

	float I;
	pid->acc = pid->acc + error * pid->Ki * pid->T;
	I = pid->acc;

	float D;
	D = ((error - pid->last_error) * pid->Kd) / pid->T;
	pid->last_error = error;

	float out;
	out = P + I + D;

	return out;
}
