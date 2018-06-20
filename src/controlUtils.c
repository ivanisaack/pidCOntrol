/*
 * controlUtils.c
 *
 *  Created on: 20 de jun. de 2018
 *      Author: ivan
 */

#include "controlUtils.h"

void tempExtrControl(control_pid_t *pid,control_variable_t *tempExtrControlVar, float tempTarget) {

	tempExtrControlVar->E = tempTarget - tempExtrControlVar->B;
	//		stdioPrintf(UART_USB, "error: %f\r\n",(uint16_t)E );

	tempExtrControlVar->X = PID_Process(pid,tempExtrControlVar->E);
	tempExtrControlVar->X = (3.3 < tempExtrControlVar->X) ? 3.3 : tempExtrControlVar->X;
	//	stdioPrintf(UART_USB, "X: %d\r\n",(uint16_t) X );

	dacWrite(DAC, (uint16_t) (tempExtrControlVar->X * (1023 / 3.3)));


	tempExtrControlVar->B = ((float) (adcRead(CH1)) * 3.3) / 1023;

}
